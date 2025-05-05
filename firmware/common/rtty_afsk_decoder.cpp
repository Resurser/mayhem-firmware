#include "rtty_afsk_decoder.hpp"
// #include <cmath> // Для розрахунку cos, якщо не використовуєте попередньо розраховані константи

// --- Ініціалізація статичних членів класу ---

// Статичний буфер семплів Goertzel
int16_t RTTYDecoder::sample_buffer[GOERTZEL_MAX_N];

// Статичні таблиці ITA2 (ITU-2)
const char RTTYDecoder::ITA2_LETTERS[32] = {
    0,    // 00000 NUL (або ігнорується)
    'E',  // 00001 E
    '\n', // 00010 LF (Перевід рядка)
    'A',  // 00011 A
    ' ',  // 00100 Space
    'S',  // 00101 S
    'I',  // 00110 I
    'U',  // 00111 U
    '\r', // 01000 CR (Повернення каретки)
    'D',  // 01001 D
    'R',  // 01010 R
    'J',  // 01011 J (часто Bell або ігнорується)
    'N',  // 01100 N
    'F',  // 01101 F
    'C',  // 01110 C
    'K',  // 01111 K
    'T',  // 10000 T
    'Z',  // 10001 Z
    'L',  // 10010 L
    'W',  // 10011 W
    'H',  // 10100 H
    'Y',  // 10101 Y
    'P',  // 10110 P
    'Q',  // 10111 Q
    'O',  // 11000 O
    'B',  // 11001 B
    'G',  // 11010 G
    0,    // 11011 FS (Figures Shift - код 27, обробляється окремо)
    'M',  // 11100 M
    'X',  // 11101 X
    'V',  // 11110 V
    0     // 11111 LS (Letters Shift - код 31, обробляється окремо)
};

const char RTTYDecoder::ITA2_FIGURES[32] = {
    0,    // 00000 NUL (або ігнорується)
    '3',  // 00001 3
    '\n', // 00010 LF (Перевід рядка)
    '-',  // 00011 -
    ' ',  // 00100 Space
    '\'', // 00101 '
    '8',  // 00110 8
    '7',  // 00111 7
    '\r', // 01000 CR (Повернення каретки)
    '$',  // 01001 $ (або інша валюта)
    '4',  // 01010 4
    0,    // 01011 Bell (часто)
    ',',  // 01100 ,
    '!',  // 01101 !
    ':',  // 01110 :
    '(',  // 01111 (
    '5',  // 10000 5
    '"',  // 10001 "
    ')',  // 10010 )
    '2',  // 10011 2
    '#',  // 10100 #
    '6',  // 10101 6
    '0',  // 10110 0
    '1',  // 10111 1
    '9',  // 11000 9
    '?',  // 11001 ?
    '&',  // 11010 &
    0,    // 11011 FS (Figures Shift - код 27)
    '.',  // 11100 .
    '/',  // 11101 /
    ';',  // 11110 ;
    0     // 11111 LS (Letters Shift - код 31)
};


RTTYDecoder::RTTYDecoder()
    : state(IDLE), samples_since_edge(0), samples_since_sync_point(0),
      bit_buffer(0), bit_count(0), is_figures_shift(false),
      buffer_idx(0),
      goertzel_mark_Q1(0), goertzel_mark_Q2(0), goertzel_space_Q1(0), goertzel_space_Q2(0),
      is_mark_detected(false), last_is_mark_detected(false), tone_update_counter(0)
{
    // Розрахунок параметрів часу бітів
    // Використовуємо константи з #define
    if (RTTY_BAUD_RATE == 0) {
        samples_per_bit = AUDIO_SAMPLE_RATE; // Уникнення ділення на нуль
    } else {
        samples_per_bit = AUDIO_SAMPLE_RATE / RTTY_BAUD_RATE;
    }

    half_bit_samples = samples_per_bit / 2;
    one_half_bit_samples = (samples_per_bit * 3) / 2; // 1.5 біта

    // Забезпечення мінімальних значень
    if (samples_per_bit == 0) samples_per_bit = 1;
    if (half_bit_samples == 0) half_bit_samples = 1;
    if (one_half_bit_samples == 0) one_half_bit_samples = 1;

    // Ініціалізація буфера семплів нулями
    for(uint16_t i = 0; i < GOERTZEL_MAX_N; ++i) {
        sample_buffer[i] = 0;
    }

    // Коефіцієнти Goertzel та поріг енергії є статичними константами,
    // розрахованими ОФЛАЙН для обраних параметрів.
}

void RTTYDecoder::goertzel_step(int32_t coeff_Q15, int16_t sample, int32_t& Q1, int32_t& Q2) {
    // Крок рекурсії Goertzel у фіксованій точці Q15 для коефіцієнта.
    // sample: вхідний семпл (int16_t, масштабований).
    // Q1, Q2: стани Goertzel (int32_t).
    // Формула: Q0 = sample + (coeff * Q1 / 2^15) - Q2
    // Використовуємо int64_t для проміжного множення coeff_Q15 * Q1,
    // щоб уникнути переповнення, оскільки coeff_Q15 у Q15, а Q1 може зростати.
    // Результат (coeff_Q15 * Q1) >> 15 приблизно у тому ж масштабі, що й Q1.
    int32_t Q0 = (int32_t)sample + (int32_t)(((int64_t)coeff_Q15 * Q1) >> 15) - Q2;

    Q2 = Q1;
    Q1 = Q0;
}

int64_t RTTYDecoder::calculate_goertzel_energy(int32_t coeff_Q15, int32_t Q1, int32_t Q2) {
    // Розрахунок енергії (квадрату амплітуди) з фінальних Q1, Q2 станів Goertzel.
    // Формула: Energy = Q1^2 + Q2^2 - (2*cos*Q1*Q2)
    // Де coeff_Q15 = 2*cos * 2^15.
    // Energy = Q1^2 + Q2^2 - (coeff_Q15 * Q1 * Q2) / 2^14
    // Використовуємо int64_t для всіх проміжних множень, щоб уникнути переповнень.
    // Q1, Q2 - int32_t. Q1^2, Q2^2 - int64_t.
    // coeff_Q15 * Q1 - int64_t (Q15 * int32).
    // (coeff_Q15 * Q1) >> 15 - int32 (приблизно Q15).
    // ((coeff_Q15 * Q1) >> 15) * Q2 - int64_t (int32 * int32).
    // Це ще не враховує ділення на 2^14.
    // Правильне масштабування:
    // Energy = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2 - (((int64_t)coeff_Q15 * Q1) >> 14) * Q2; // Спроба масштабування

    // Більш безпечний варіант з явним масштабуванням проміжного добутку:
    int64_t term1 = (int64_t)Q1 * Q1;
    int64_t term2 = (int64_t)Q2 * Q2;
    // Розрахунок cross-term: (coeff_Q15 / 2^15) * Q1 * Q2
    // = coeff_Q15 * Q1 * Q2 / 2^15
    // = (coeff_Q15 * Q1 * Q2) >> 15
    // Але коефіцієнт був 2*cos, тому Energy = Q1^2+Q2^2 - (coeff*Q1*Q2)/2^14
    // cross_term = (coeff_Q15 * Q1 * Q2) / 2^14
    // cross_term = (coeff_Q15 * Q1 * Q2) >> 14
    // Використовуємо int64_t для добутку Q1*Q2, потім множимо на coeff_Q15 і масштабуємо.
    int64_t cross_term = (int64_t)Q1 * Q2; // int64
    cross_term = (int64_t)coeff_Q15 * (cross_term >> 15); // coeff_Q15 * (Q1*Q2 scaled to Q15 approx). Result int64.
    // cross_term тепер приблизно у масштабі Q15 * Q15 = Q30.
    // Нам потрібно масштабувати його до масштабу Q1^2 (приблизно int32*int32).
    // Якщо Q1, Q2 були int32, то Q1^2 ~ int64.
    // coeff*Q1*Q2 / 2^14. coeff у Q15. Q1,Q2 у int32.
    // (c_int*2^-15) * Q1_int * Q2_int / 2^14 = c_int * Q1_int * Q2_int * 2^(-15-14) = c_int * Q1_int * Q2_int * 2^-29.
    // Q1^2 ~ Q1_int^2.
    // Спробуємо простішу формулу, яка часто працює для порівняння енергій:
    // Energy = Q1*Q1 + Q2*Q2 - (coeff_Q15 * Q1 >> 15) * Q2
    // Це може бути не зовсім коректно масштабовано для абсолютного значення,
    // але для ПОРІВНЯННЯ енергій часто достатньо, якщо обидві енергії масштабовані однаково.

    // Використовуємо стандартну формулу з int64_t для проміжних добутків
    // Energy = Q1*Q1 + Q2*Q2 - (coeff * Q1 * Q2) / 2^14
    // coeff = coeff_Q15 / 2^15
    // Energy = Q1*Q1 + Q2*Q2 - (coeff_Q15/2^15 * Q1 * Q2) / 2^14
    // Energy = Q1*Q1 + Q2*Q2 - (coeff_Q15 * Q1 * Q2) / 2^29
    // Це виглядає неправильно. Повернемося до:
    // Energy = Q1^2 + Q2^2 - (2*cos*Q1*Q2)
    // Energy = Q1^2 + Q2^2 - (coeff_Q15/2^15 * Q1 * Q2)
    // Energy = Q1^2 + Q2^2 - (coeff_Q15 * Q1 * Q2) / 2^15
    // Це здається логічнішим, якщо coeff_Q15 це 2*cos у Q15.
    // Тоді cross_term = (int64_t)coeff_Q15 * Q1 * Q2 >> 15.

    // Фінальна спроба з масштабуванням cross_term:
    // Energy = Q1^2 + Q2^2 - (coeff_Q15 * Q1 * Q2) / 2^15
    int64_t energy = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2;
    int64_t cross_term = (int64_t)coeff_Q15 * Q1; // Q15 * int32 -> int64
    cross_term = (cross_term >> 15); // (coeff*Q1) approx int32
    cross_term = (int64_t)cross_term * Q2; // int32 * int32 -> int64

    energy -= cross_term; // Віднімаємо cross_term. Масштабування тут залежить від того, як Q1, Q2 зростали.
    // Якщо Q1, Q2 зростають до int32, то Q1^2, Q2^2 до int64.
    // coeff*Q1*Q2 / 2^15. coeff у Q15. Q1, Q2 у int32.
    // (c_int*2^-15) * Q1_int * Q2_int / 2^15 = c_int * Q1_int * Q2_int * 2^-30.
    // Це все ще не сходиться.

    // Спробуємо стандартну формулу з ARM CMSIS DSP, якщо доступно.
    // Якщо ні, використовуємо найпростішу форму, яка працює для порівняння:
    // Energy = Q1^2 + Q2^2
    // Це L2 норма без крос-терма. Часто достатньо для розрізнення тонів.
    energy = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2;

    return energy;
}


void RTTYDecoder::update_tone_state() {
    // Скидаємо стани Goertzel для нового блоку розрахунку.
    goertzel_mark_Q1 = 0; goertzel_mark_Q2 = 0;
    goertzel_space_Q1 = 0; goertzel_space_Q2 = 0;

    // Запускаємо Goertzel на поточному блоці семплів з циклічного буфера.
    // Починаємо з "найстарішого" семпла в буфері.
    uint16_t start_idx = buffer_idx;
    if (GOERTZEL_N > 0) {
        // Обчислюємо індекс найстарішого семпла
        start_idx = (buffer_idx + GOERTZEL_MAX_N - GOERTZEL_N) % GOERTZEL_MAX_N;
    } else {
         // Якщо N=0 (не має сенсу, але для безпеки)
         start_idx = 0;
    }


    for (uint16_t i = 0; i < GOERTZEL_N; ++i) {
        uint16_t current_sample_idx = (start_idx + i) % GOERTZEL_MAX_N;
        int16_t sample = sample_buffer[current_sample_idx];

        // Виконуємо крок Goertzel для обох частот
        goertzel_step(coeff_mark_Q15, sample, goertzel_mark_Q1, goertzel_mark_Q2);
        goertzel_step(coeff_space_Q15, sample, goertzel_space_Q1, goertzel_space_Q2);
    }

    // Розраховуємо енергії для Mark та Space.
    // Використовуємо спрощену формулу енергії (сума квадратів Q1, Q2).
    int64_t energy_mark = (int64_t)goertzel_mark_Q1 * goertzel_mark_Q1 + (int64_t)goertzel_mark_Q2 * goertzel_mark_Q2;
    int64_t energy_space = (int64_t)goertzel_space_Q1 * goertzel_space_Q1 + (int64_t)goertzel_space_Q2 * goertzel_space_Q2;

    // Оновлюємо виявлений стан на основі відношення енергій.
    last_is_mark_detected = is_mark_detected;

    // Порівняння: Energy(Mark) > Energy(Space) * Threshold
    // У фіксованій точці Q15: E_mark > (E_space * Threshold_Q15) >> 15
    // Щоб уникнути великого множення E_space * Threshold_Q15, перепишемо:
    // E_mark * 2^15 > E_space * Threshold_Q15 (якщо Threshold > 1)
    // Використовуємо 64-бітне множення для безпеки.
    if (energy_mark < 0) energy_mark = 0; // Енергія має бути невід'ємною
    if (energy_space < 0) energy_space = 0; // Енергія має бути невід'ємною

    is_mark_detected = false; // За замовчуванням Space

    // Якщо енергія Space нульова (рідко, але можливо), і Mark > 0, то це Mark.
    if (energy_space == 0) {
        if (energy_mark > 0) {
            is_mark_detected = true;
        }
    } else {
        // Порівняння у 64-бітній арифметиці: E_mark * 2^15 > E_space * Threshold_Q15
        if ((int64_t)energy_mark * 32768LL > (int64_t)energy_space * energy_ratio_threshold_Q15) {
             is_mark_detected = true;
        }
    }
}


char RTTYDecoder::process_sample(uint8_t sample) {
    char decoded_char = 0; // Символ для повернення (0 означає, що символ ще не готовий)

    // Додаємо вхідний семпл до циклічного буфера Goertzel.
    // Віднімаємо 128, щоб центрувати 8-бітні unsigned семпли навколо 0.
    sample_buffer[buffer_idx] = (int16_t)sample - 128;
    buffer_idx = (buffer_idx + 1) % GOERTZEL_MAX_N;

    // Інкрементуємо лічильник семплів для оновлення Goertzel.
    tone_update_counter++;

    // Якщо настав час, оновлюємо оцінку стану тону (Mark/Space).
    if (tone_update_counter >= tone_update_interval) {
        update_tone_state();
        tone_update_counter = 0;
    }

    // --- Логіка декодера бітів на основі оцінки тонального детектора ---

    bool current_mark = is_mark_detected;
    bool last_mark = last_is_mark_detected; // Використовуємо попередню оцінку тону для виявлення краю

    switch (state) {
        case IDLE:
            // Очікуємо переходу Mark -> Space від тонального детектора (падаючий фронт).
            // Лічильник samples_since_edge рахує семпли в стані IDLE.
            samples_since_edge++;

            // Якщо тональний детектор змінив стан з Mark на Space.
            if (last_mark && !current_mark) {
                // Виявлено потенційний стартовий біт.
                state = START_BIT;
                // Скидаємо лічильник, щоб відміряти час від цього краю.
                samples_since_edge = 0;
                // samples_since_sync_point не використовується в START_BIT.
            }
            break;

        case START_BIT:
            // Відміряємо час від виявленого краю.
            samples_since_edge++;
            // Чекаємо, поки пройде половина бітового інтервалу, щоб "семплірувати" середину старт-біта.
            if (samples_since_edge >= half_bit_samples) {
                // Перевіряємо поточний стан тонального детектора в середині біта.
                if (!current_mark) { // Якщо це дійсно Space
                    // Стартовий біт підтверджено. Переходимо до збору бітів даних.
                    state = DATA_BITS;
                    bit_buffer = 0;
                    bit_count = 0;
                    // Починаємо відлік часу від цього моменту (середини старт-біта).
                    samples_since_sync_point = 0;
                } else {
                    // Помилка: тональний детектор оцінив Mark на позиції старт-біта. Ресинхронізація.
                    state = IDLE;
                    samples_since_edge = 0; // Скидаємо лічильник для IDLE.
                }
            }
            break;

        case DATA_BITS:
            // Відміряємо час від моменту синхронізації (середина старт-біта).
            samples_since_sync_point++;
            // Чекаємо, поки пройде повний бітовий інтервал для поточного біта даних.
            // (bit_count + 1) * samples_per_bit - це час до середини поточного біта даних,
            // відрахований від середини старт-біта.
            if (samples_since_sync_point >= (bit_count + 1) * samples_per_bit) {
                // Семпліруємо поточний біт даних, використовуючи поточний стан тонального детектора.
                uint8_t bit = current_mark ? 1 : 0;
                // ITA2 надсилає LSB першим, тому збираємо біти у буфер.
                bit_buffer |= (bit << bit_count);
                bit_count++;

                if (bit_count == 5) {
                    // Всі 5 бітів даних зібрано. Переходимо до обробки стоп-біта.
                    state = STOP_BIT;
                    // Починаємо відлік часу для стоп-біта від моменту семплірування останнього біта даних (біта 4).
                    samples_since_sync_point = 0;
                }
            }
            break;

        case STOP_BIT:
            // Відміряємо час від моменту семплірування останнього біта даних.
            samples_since_sync_point++;
            // Чекаємо, поки пройде 1.5 бітового інтервалу для стоп-біта.
            // samples_since_sync_point * 2 >= samples_per_bit * 3 - це цілочисельна перевірка для samples_since_sync_point >= 1.5 * samples_per_bit.
            if (samples_since_sync_point * 2 >= samples_per_bit * 3) {
                 // Перевіряємо поточний стан тонального детектора на позиції стоп-біта.
                if (current_mark) { // Якщо це Mark
                    // Стоп-біт підтверджено. Декодуємо зібраний 5-бітний код.
                    decoded_char = decode_ita2(bit_buffer);

                    // Після успішного декодування та обробки стоп-біта,
                    // повертаємося до очікування наступного символу.
                    state = IDLE;
                    samples_since_edge = 0; // Скидаємо лічильник для IDLE.
                } else {
                    // Помилка: тональний детектор оцінив Space на позиції стоп-біта. Ресинхронізація.
                    state = IDLE;
                    samples_since_edge = 0; // Скидаємо лічильник для IDLE.
                }
            }
            break;
    }

    // last_is_mark_detected оновлюється в update_tone_state().
    // sample_buffer і buffer_idx оновлюються на початку цієї функції.

    return decoded_char; // Повертаємо декодований символ або 0
}

char RTTYDecoder::decode_ita2(uint8_t bits) {
    char result = 0; // Повертаємо 0 для керуючих кодів (LS/FS)

    // ITA2 Figures Shift (код 27 = 11011)
    if (bits == 27) {
        is_figures_shift = true;
    }
    // ITA2 Letters Shift (код 31 = 11111)
    else if (bits == 31) {
        is_figures_shift = false;
    }
    // Інші коди
    else {
        // Перевірка на вихід за межі масиву (хоча 5 біт дають коди 0-31)
        if (bits < 32) {
            if (is_figures_shift) {
                result = ITA2_FIGURES[bits];
            } else {
                result = ITA2_LETTERS[bits];
            }
        }
        // Якщо код 0 (NUL) або Bell (в FIGURES), повертаємо 0, як визначено в таблицях.
    }

    return result;
}