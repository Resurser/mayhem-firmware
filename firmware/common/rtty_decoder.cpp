#include "rtty_decoder.hpp"
// #include <cmath> // Для cos (якщо розраховуєте коефіцієнти під час компіляції)
// #define _USE_MATH_DEFINES // Для M_PI


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

// Конструктор класу
RTTYDecoder::RTTYDecoder(uint32_t sample_rate, uint16_t baud_rate, uint16_t f_space, uint16_t f_mark, uint16_t goertzel_n, float energy_ratio_threshold)
    : sample_rate(sample_rate), baud_rate(baud_rate), f_space(f_space), f_mark(f_mark), goertzel_n(goertzel_n),
      state(IDLE), samples_since_edge(0), samples_since_sync_point(0),
      bit_buffer(0), bit_count(0), is_figures_shift(false),
      buffer_idx(0), // Ініціалізація індексу циклічного буфера
      goertzel_mark_Q1(0), goertzel_mark_Q2(0), goertzel_space_Q1(0), goertzel_space_Q2(0), // Ініціалізація станів Goertzel
      is_mark_detected(false), last_is_mark_detected(false), tone_update_counter(0) // Ініціалізація станів тонального детектора
{
    // Перевірка та обмеження розміру буфера Goertzel
    if (goertzel_n == 0 || goertzel_n > GOERTZEL_MAX_N) {
        goertzel_n = GOERTZEL_MAX_N; // Використовуємо максимальний або стандартний розмір
    }

    // Ініціалізація статичного буфера нулями
    for(uint16_t i = 0; i < GOERTZEL_MAX_N; ++i) {
        sample_buffer[i] = 0;
    }

    // Розрахунок параметрів часу бітів
    if (baud_rate == 0) baud_rate = 1; // Уникнення ділення на нуль
    samples_per_bit = sample_rate / baud_rate;
    half_bit_samples = samples_per_bit / 2;
    one_half_bit_samples = (samples_per_bit * 3) / 2;

    // Захист від нульових значень (хоча для типових Fs/Baud малоймовірно)
    if (samples_per_bit == 0) samples_per_bit = 1;
    if (half_bit_samples == 0) half_bit_samples = 1;
    if (one_half_bit_samples == 0) one_half_bit_samples = 1;


    // Розрахунок коефіцієнтів Goertzel у фіксованій точці Q15
    // k = f * N / Fs
    float k_space = (float)f_space * goertzel_n / sample_rate;
    float k_mark = (float)f_mark * goertzel_n / sample_rate;

    // ЦІ ЗНАЧЕННЯ ПОТРІБНО РОЗРАХУВАТИ ТОЧНО ОФЛАЙН для ваших Fs=24000, N=256, f_space=1275, f_mark=1445
    // Використання cmath тут лише для ілюстрації. На M0 ви маєте використовувати готові константи.
    // Приклад розрахунку коефіцієнта coeff = 2 * cos(2*PI*k/N). Масштабуємо на 2^15 для Q15.
    // Якщо Fs=24000, N=256, f_space=1275 (k=13.6), f_mark=1445 (k=15.4167):
    // coeff_space = 2 * cos(2*PI*13.6/256) ~= 1.8909  => Q15: ~ (int32_t)(1.8909 * 32767.5) = 61967
    // coeff_mark  = 2 * cos(2*PI*15.4167/256) ~= 1.8603  => Q15: ~ (int32_t)(1.8603 * 32767.5) = 60964
    // Замініть ці приклади константами, розрахованими для ваших точних параметрів:
    coeff_space_Q15 = 61967; // ПРИКЛАД константи Goertzel Q15 для 1275 Гц @ 24кГц, N=256
    coeff_mark_Q15 = 60964;  // ПРИКЛАД константи Goertzel Q15 для 1445 Гц @ 24кГц, N=256


    // Поріг відношення енергій у Q15. energy_ratio_threshold * 2^15.
    energy_ratio_threshold_Q15 = (int32_t)(energy_ratio_threshold * 32767.5f);
     // Гарантуємо, що поріг >= 1.0 (в Q15 це >= 32768)
    if (energy_ratio_threshold_Q15 < 32768) energy_ratio_threshold_Q15 = 32768;


    // Як часто оновлювати оцінку тону Goertzel (в семплах).
    // Оновлюємо кожні N/4 або N/8 семплів. Частіше оновлення = менша затримка, більше обчислень.
    tone_update_interval = goertzel_n / 4;
    if (tone_update_interval == 0) tone_update_interval = 1; // Мінімум 1 семпл
}

// Метод виконує один крок рекурсії Goertzel у фіксованій точці.
// Q1, Q2 оновлюються за посиланням.
void RTTYDecoder::goertzel_step(int32_t coeff_Q15, int16_t sample, int32_t& Q1, int32_t& Q2) {
    // Рекурсія: Q[n] = sample[n] + coeff * Q[n-1] - Q[n-2]
    // sample[n] - вхідний семпл (int16_t, центрований).
    // coeff_Q15 - коефіцієнт (int32_t у Q15).
    // Q1 = Q[n-1], Q2 = Q[n-2] (int32_t).

    // Розрахунок coeff * Q[n-1] у фіксованій точці:
    // coeff_Q15 (Q15) * Q1 (int32). Якщо Q1 є Q_int * 2^q, то добуток (coeff_int*Q1_int) * 2^(q-15).
    // Якщо Q1 також масштабований на 2^15 (якщо вхідні семпли були масштабовані до Q15),
    // то Q15 * Q15 дасть Q30. Тоді зсув >> 15 дасть Q15.
    // Якщо Q1 - це просто int32_t, який може зростати, то його масштаб змінюється.
    // При N=256 і int16 вході (-128..127), Q1, Q2 не зростають настільки сильно,
    // щоб int32 не вистачило для самих Q1, Q2.
    // Але добуток coeff_Q15 * Q1 (int32 * int32) може тимчасово перевищити межі int32.
    // Тому використовуємо int64_t для проміжного добутку:
    int32_t Q0 = (int32_t)sample + (int32_t)(((int64_t)coeff_Q15 * Q1) >> 15) - Q2;

    Q2 = Q1;
    Q1 = Q0;
}

// Метод розраховує енергію |Y(k)|^2 з фінальних Q1, Q2 станів Goertzel.
// Повертає енергію у форматі int64_t.
int64_t RTTYDecoder::calculate_goertzel_energy(int32_t coeff_Q15, int32_t Q1, int32_t Q2) {
    // Енергія |Y(k)|^2 = Q1^2 + Q2^2 - (2*cos(w0))*Q1*Q2
    // де coeff_Q15 = (2*cos(w0)) * 2^15.
    // Енергія = Q1^2 + Q2^2 - (coeff_Q15/2^15) * Q1 * Q2
    // Енергія = Q1^2 + Q2^2 - (coeff_Q15 * Q1 * Q2) / 2^15
    // Оскільки Q1, Q2 це int32, а coeff_Q15 це Q15 (int32),
    // добуток coeff_Q15 * Q1 може бути int64. Добуток Q1*Q2 може бути int64.
    // Фінальний розрахунок з int64_t:
    int64_t term1 = (int64_t)Q1 * Q1; // Q1^2
    int64_t term2 = (int64_t)Q2 * Q2; // Q2^2

    // Член з коефіцієнтом: (coeff_Q15 * Q1 * Q2) / 2^15
    // (coeff_Q15 * Q1) це Q15 * int32, може бути int64.
    // (((int64_t)coeff_Q15 * Q1) >> 15) це приблизно (coeff * Q1) як int32.
    // Потім множимо на Q2 (int32). Результат може бути int64.
    // scaling: coeff_Q15 (Q15), Q1 (int32 scale ~Q15), Q2 (int32 scale ~Q15)
    // coeff*Q1*Q2 scale is Q15 * Q15 * Q15 = Q45 if everything was Q15 scaled.
    // With int32 Q1, Q2, scale is more like Q15 * 2^q * 2^q = 2^(2q+15).
    // Let's use the standard formula implementation with explicit int64 casts.
    // Енергія = Q1*Q1 + Q2*Q2 - ((int64_t)coeff_Q15 * Q1 * Q2) / 2^14 - це неправильно масштабування
    // Енергія = Q1*Q1 + Q2*Q2 - ((int64_t)coeff_Q15 * Q1 * Q2) / 2^15
    // E = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2 - (((int64_t)coeff_Q15 * Q1) >> 15) * Q2; // це все ще не вірно масштабування

    // Коректне масштабування для стандартної формули |Y(k)|^2 = Q1^2 + Q2^2 - 2*cos*Q1*Q2
    // де coeff_Q15 = 2*cos*2^15.
    // Член з коефіцієнтом має масштаб, схожий на Q1^2 або Q2^2.
    // Припустимо Q1, Q2 мають масштаб 2^S. Тоді Q1^2 ~ (2^S)^2 = 2^2S.
    // 2*cos*Q1*Q2 ~ (coeff_Q15/2^15) * (Q1_int*2^S) * (Q2_int*2^S) = coeff_Q15_int * Q1_int*Q2_int * 2^(2S-15).
    // Щоб вирівняти масштаби, потрібно домножити член з коефіцієнтом на 2^15.
    // Енергія = Q1^2 + Q2^2 - (coeff_Q15 * Q1 * Q2) / 2^0 ... якщо coeff_Q15 * Q1 * Q2 вже має масштаб як Q1^2 * 2^15.
    // Найбезпечніше: Енергія = Q1^2 + Q2^2 - (coeff_Q15 * Q1/2^15 * Q2/2^15) * 2^30
    // Енергія = Q1^2 + Q2^2 - (coeff_Q15 * Q1 * Q2) * 2^0 ... Ні.
    // Енергія = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2 - (((int64_t)coeff_Q15 * ((int64_t)Q1 * Q2 >> 15)) >> 15); // Це виглядає правильно, Q1*Q2 scaled to Q15, then mult by coeff_Q15, then scaled again

    int64_t energy = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2;
    // Розрахунок перехресного члена: (coeff_Q15 * Q1 * Q2) / 2^15
    // Використовуємо інший порядок множення, щоб уникнути величезних проміжних значень, якщо можливо.
    // Або просто використовувати int64_t для всього:
    int64_t cross_term = (int64_t)coeff_Q15 * Q1; // Q15 * int32 = int64
    cross_term = (cross_term >> 15); // scale back by 2^15. Result is int32 approx.
    cross_term = (int64_t)cross_term * Q2; // int32 * int32 = int64

    energy -= (cross_term >> 0); // Фінальне масштабування для енергії. Якщо Q1,Q2 int32, то cross_term вже має схожий масштаб до Q1*Q1.

    // Найнадійніший варіант - використовувати стандартну формулу з int64:
    energy = (int64_t)Q1 * Q1 + (int64_t)Q2 * Q2 - (((int64_t)coeff_Q15 * Q1) >> 14) * Q2; // Спробуємо це масштабування (ділення на 2^14)

    return energy;
}

// Метод оновлює стан тонального детектора, запускаючи Goertzel.
void RTTYDecoder::update_tone_state() {
    // Скидаємо стани Goertzel для нового розрахунку на поточному блоці
    goertzel_mark_Q1 = 0; goertzel_mark_Q2 = 0;
    goertzel_space_Q1 = 0; goertzel_space_Q2 = 0;

    // Виконуємо Goertzel на поточному вмісті циклічного буфера.
    // Буфер зберігає останні `goertzel_n` семплів.
    // Починаємо читати з позиції buffer_idx (це найстаріший семпл, якщо буфер повний),
    // і читаємо goertzel_n семплів по колу.
    uint16_t start_idx = buffer_idx; // buffer_idx вказує на місце для НАСТУПНОГО запису. Найстаріший семпл - тут.
    // Якщо buffer_idx == 0, найстаріший семпл в goertzel_n-1.
    // Правильний старт індексу: buffer_idx є місцем для нового, отже N семплів назад по колу.
    start_idx = (buffer_idx + GOERTZEL_MAX_N - goertzel_n) % GOERTZEL_MAX_N;


    for (uint16_t i = 0; i < goertzel_n; ++i) {
        uint16_t current_sample_idx = (start_idx + i) % GOERTZEL_MAX_N;
        int16_t sample = sample_buffer[current_sample_idx];

        // Виконуємо кроки Goertzel для обох частот
        goertzel_step(coeff_mark_Q15, sample, goertzel_mark_Q1, goertzel_mark_Q2);
        goertzel_step(coeff_space_Q15, sample, goertzel_space_Q1, goertzel_space_Q2);
    }

    // Розраховуємо енергії для Mark та Space тонів
    int64_t energy_mark = calculate_goertzel_energy(coeff_mark_Q15, goertzel_mark_Q1, goertzel_mark_Q2);
    int64_t energy_space = calculate_goertzel_energy(coeff_space_Q15, goertzel_space_Q1, goertzel_space_Q2);

    // Оновлюємо detected стан на основі порівняння енергій
    last_is_mark_detected = is_mark_detected;

    // Порівняння: E_mark > E_space * Threshold
    // У фіксованій точці: E_mark > (E_space * Threshold_Q15) >> 15
    // Щоб уникнути потенційно великого проміжного добутку (int64 * int32),
    // переформулюємо: E_mark * 2^15 > E_space * Threshold_Q15
    // Використовуємо 32768LL для 2^15 як long long
    // Перевірка на нульову енергію Space, щоб уникнути ділення на нуль або неправильного порівняння
    if (energy_mark < 0) energy_mark = 0; // Енергія не може бути від'ємною
    if (energy_space < 0) energy_space = 0;

    is_mark_detected = (energy_mark > 0 && // Mark detected тільки якщо є якась енергія на Mark частоті
                        (energy_space == 0 || // Або якщо енергія Space дорівнює нулю (чи дуже мала)
                         // Або якщо E_mark * 2^15 > E_space * Threshold_Q15
                         (int64_t)energy_mark * 32768LL > (int64_t)energy_space * energy_ratio_threshold_Q15) );

}


// Основний метод обробки одного вхідного аудіо семпла.
// sample: 8-бітний беззнаковий семпл (0-255).
// Повертає декодований символ або 0.
char RTTYDecoder::process_sample(uint8_t sample) {
    char decoded_char = 0; // Символ для повернення (0 означає, що символ ще не готовий)

    // Додаємо вхідний семпл до циклічного буфера Goertzel.
    // Перетворюємо 8-бітний беззнаковий [0, 255] у 16-бітний знаковий [-128, 127].
    // Це центрований сигнал, що потрібно для Goertzel.
    sample_buffer[buffer_idx] = (int16_t)sample - 128;
    buffer_idx = (buffer_idx + 1) % GOERTZEL_MAX_N; // Переходимо до наступної позиції, зациклюємо

    // Оновлюємо лічильник для періодичного запуску Goertzel.
    tone_update_counter++;

    // Якщо настав час, запускаємо повний розрахунок Goertzel на буфері
    // та оновлюємо detected стан Mark/Space.
    if (tone_update_counter >= tone_update_interval) {
        update_tone_state();
        tone_update_counter = 0; // Скидаємо лічильник
    }

    // --- Логіка декодера бітів (використовує стан тонального детектора) ---

    // Використовуємо поточну та попередню оцінку стану тону від детектора.
    bool current_mark = is_mark_detected;
    bool last_mark = last_is_mark_detected;

    switch (state) {
        case IDLE:
            // В стані IDLE ми чекаємо на початок нового символу,
            // який сигналізується переходом з Mark на Space.
            // Цей перехід ми ловимо за зміною стану тонального детектора.
            samples_since_edge++; // Рахуємо семпли в стані очікування

            // Виявляємо падаючий фронт detected тону (Mark -> Space)
            if (last_mark && !current_mark) {
                // Виявлено початок стартового біта. Переходимо до його обробки.
                state = START_BIT;
                samples_since_edge = 0; // Скидаємо лічильник, відлік від моменту виявлення краю
                // samples_since_sync_point не використовується в START_BIT
            }
            // Якщо тональний детектор змінився на Space, але не з Mark (наприклад, був шум),
            // або якщо це Mark -> Mark або Space -> Space, залишаємося в IDLE.
            break;

        case START_BIT:
            // Обробка стартового біта. Він має бути Space.
            samples_since_edge++; // Рахуємо семпли з моменту виявлення краю

            // Семпліруємо (зчитуємо стан тонального детектора) посередині стартового біта
            // (через 0.5 біта від моменту виявлення краю).
            if (samples_since_edge >= half_bit_samples) {
                // У цей момент ми дивимося на поточну оцінку тонального детектора.
                if (!current_mark) { // Перевіряємо, чи детектор оцінює Space
                    // Стартовий біт підтверджено як Space. Переходимо до збору бітів даних.
                    state = DATA_BITS;
                    bit_buffer = 0; // Очищаємо буфер для нових бітів
                    bit_count = 0;  // Скидаємо лічильник бітів
                    // Скидаємо лічильник, відлік від моменту успішного семплірування середини старт-біта
                    samples_since_sync_point = 0;
                } else {
                    // Помилка: тональний детектор оцінив Mark замість Space посередині старт-біта.
                    // Це або шум, або помилка синхронізації. Повертаємося в IDLE для ресинхронізації.
                    state = IDLE;
                    samples_since_edge = 0; // Скидаємо лічильник IDLE
                }
            }
            break;

        case DATA_BITS:
            // Збір 5 бітів даних.
            samples_since_sync_point++; // Рахуємо семпли з моменту семплірування середини старт-біта

            // Семпліруємо середину кожного біта даних (через 0.5 + N * 1.0 біта від старт-біта).
            // Тут ми використовуємо samples_since_sync_point, який почав відлік після 0.5 біта старт-біта.
            // Точки семплірування для даних бітів (від моменту семплірування старт-біта): 1.0, 2.0, 3.0, 4.0, 5.0 бітів.
            // Відповідно до samples_since_sync_point (який рахує від 0.5 біта): 0.5, 1.5, 2.5, 3.5, 4.5 бітів.
            // Тому перевірка: samples_since_sync_point >= (bit_count + 0.5) * samples_per_bit.
            // У цілих числах: samples_since_sync_point * 2 >= (bit_count + 0.5) * samples_per_bit * 2
            // samples_since_sync_point * 2 >= (2*bit_count + 1) * samples_per_bit
            // Наприклад: біт 0: samples_since_sync_point * 2 >= 1 * samples_per_bit
            // біт 1: samples_since_sync_point * 2 >= 3 * samples_per_bit
            // біт 4: samples_since_sync_point * 2 >= 9 * samples_per_bit
            if (samples_since_sync_point * 2 >= (bit_count * 2 + 1) * samples_per_bit) {

                // Семпліруємо поточний біт даних, використовуючи поточну оцінку тонального детектора.
                uint8_t bit = current_mark ? 1 : 0;
                // ITA2 надсилає LSB першим, тому збираємо біти у буфер у правильному порядку.
                // bit_count йде від 0 до 4, біт 0 - LSB.
                bit_buffer |= (bit << bit_count);
                bit_count++;

                if (bit_count == 5) {
                    // Всі 5 бітів даних зібрано. Переходимо до обробки стоп-біта.
                    state = STOP_BIT;
                    // Скидаємо лічильник, відлік від моменту семплірування останнього (5-го) біта даних.
                    samples_since_sync_point = 0;
                }
            }
            break;

        case STOP_BIT:
            // Обробка 1.5-бітного стоп-коду. Він має бути Mark.
            samples_since_sync_point++; // Рахуємо семпли з моменту семплірування останнього біта даних

             // Семпліруємо середину стоп-біта (1.5 біта після кінця останнього біта даних).
             // Кінець останнього біта даних - це samples_since_sync_point = 0 у цьому стані.
             // Середина стоп-біта знаходиться на 1.5 біта пізніше.
             // Перевірка: samples_since_sync_point >= 1.5 * samples_per_bit.
             // У цілих числах: samples_since_sync_point * 2 >= samples_per_bit * 3
            if (samples_since_sync_point * 2 >= samples_per_bit * 3) {

                 // Використовуємо поточну оцінку тонального детектора.
                if (current_mark) { // Перевіряємо, чи детектор оцінює Mark
                    // Стоп-біт підтверджено як Mark. Декодуємо зібраний 5-бітний код даних.
                    decoded_char = decode_ita2(bit_buffer);

                    // Після успішного декодування та обробки стоп-біта,
                    // символ завершено. Повертаємося до очікування наступного символу.
                    state = IDLE;
                    samples_since_edge = 0; // Скидаємо лічильник IDLE для наступного символу
                } else {
                    // Помилка: тональний детектор оцінив Space на позиції стоп-біта.
                    // Це означає втрату синхронізації або сильний шум. Повертаємося в IDLE.
                    state = IDLE;
                    samples_since_edge = 0; // Скидаємо лічильник IDLE
                }
            }
            break;
    }

    // last_is_mark_detected оновлюється всередині update_tone_state(),
    // яка викликається періодично. sample_buffer і buffer_idx оновлюються на початку
    // process_sample.

    return decoded_char; // Повертаємо декодований символ (0, якщо символ ще не готовий)
}

// Метод декодує 5-бітний код ITA2.
// bits: 5-бітний код (0-31) з LSB у біті 0.
// Повертає символ або 0, якщо це код зсуву (LS/FS).
char RTTYDecoder::decode_ita2(uint8_t bits) {
    char result = 0; // Повертаємо 0 для керуючих кодів (LS/FS)

    // Перевірка на коди зсуву (Shift codes)
    // ITA2 Figures Shift (код 27 = 11011)
    if (bits == 27) {
        is_figures_shift = true;
    }
    // ITA2 Letters Shift (код 31 = 11111)
    else if (bits == 31) {
        is_figures_shift = false;
    }
    // Інші коди - декодуємо за поточною таблицею
    else {
        // Перевірка на всяк випадок, хоча 5 біт дають індекси тільки від 0 до 31
        if (bits < 32) {
            if (is_figures_shift) {
                result = ITA2_FIGURES[bits];
            } else {
                result = ITA2_LETTERS[bits];
            }
        }
        // Деякі коди (як NUL, Bell) можуть бути представлені як 0 в наших таблицях,
        // ці 0 також повертаються як 0, що є прийнятним.
    }

    return result;
}

/*
**Як використовувати з аудіофайлом на PortaPack:**

1.  **Компіляція:** Додайте ці `.h` та `.cpp` файли до початкових файлів прошивки PortaPack, яку ви компілюєте. Переконайтеся, що ваш компілятор для ARM Cortex-M0 підтримує необхідні функції (цілочисельна арифметика, `int64_t`).
2.  **Читання файлу:** У прошивці PortaPack вам потрібно буде реалізувати функціонал читання аудіоданих з файлу (наприклад, з SD-карти). PortaPack зазвичай має доступ до файлової системи.
3.  **Ініціалізація Декодера:** Коли користувач обирає декодування RTI з файлу, створіть екземпляр класу `RTIDecoder`. **Ключові параметри**:
    * `sample_rate`: ЦЕ ВАЖЛИВО! Це має бути ЧАСТОТА ДИСКРЕТИЗАЦІЇ АУДІОСИГНАЛУ В САМОМУ ФАЙЛІ. Наприклад, 24000, 48000 Гц тощо. Вам потрібно визначити це з файлу або знати, як PortaPack записує аудіо.
    * `baud_rate`: 50.
    * `f_space`: 1275 (або інша базова частота, якщо вона відома).
    * `f_mark`: 1445 (або `f_space + 170`).
    * `goertzel_n`: Розмір блоку Goertzel (наприклад, 256). Вибирайте його, враховуючи $F_s$ та обчислювальні можливості.
    * `energy_ratio_threshold`: Поріг (наприклад, 1.5).
4.  **Обробка Семплів:** Читайте 8-бітні аудіосемпли з файлу по черзі або блоками. Для кожного прочитаного 8-бітного семпла викликайте метод `process_sample()` екземпляра декодера:
    ```cpp
    // Приклад псевдокоду в циклі читання файлу в PortaPack
    // RTIDecoder my_rtty_decoder(sample_rate_из_файла, 50, 1275, 1445, 256, 1.5f);
    // while (есть_данные_в_файле) {
    //     uint8_t audio_sample = прочитать_семпл_из_файла();
    //     char decoded_char = my_rtty_decoder.process_sample(audio_sample);
    //     if (decoded_char != 0) {
    //         // Декодовано символ! Вивести його на екран PortaPack
    //         // Використовуйте функції PortaPack для виводу тексту.
    //         // display_putc(decoded_char);
    //     }
    // }
    ```
    */