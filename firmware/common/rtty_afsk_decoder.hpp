#ifndef RTTY_AFSK_DECODER_50BAUD_H
#define RTTY_AFSK_DECODER_50BAUD_H

#include <cstdint>  // Для uint8_t, uint16_t, int16_t, int32_t, int64_t
// #include <cmath> // Може знадобитися для розрахунку коефіцієнтів ОДИН РАЗ під час ініціалізації
// #define _USE_MATH_DEFINES // Для M_PI у Visual Studio, якщо використовуєте cmath

// --- Конфігураційні параметри (можна налаштувати) ---
// Частота дискретизації аудіосигналу з SDR.
// Це ПРИКЛАД. Потрібно використовувати реальну частоту дискретизації вашого аудіопайплайна PortaPack.
#define AUDIO_SAMPLE_RATE 12000

// Швидкість передачі RTTY в бодах.
#define RTTY_BAUD_RATE 50

// Частоти тонів Space та Mark. Різниця має бути 170 Гц.
// Це ПРИКЛАД типової тональної пари.
#define RTTY_F_SPACE 1275
#define RTTY_F_MARK (RTTY_F_SPACE + 170)  // 1445 Гц

// Розмір блоку для алгоритму Goertzel. Впливає на частотну роздільну здатність та обчислення.
// Більше N -> краща роздільна здатність, більше обчислень/пам'яті.
// Потрібно підібрати N так, щоб розрізняти 170 Гц з достатньою надійністю.
// N = SampleRate / (приблизна роздільна здатність). 12000 / 93.75 = 128.
#define GOERTZEL_N 128

// Поріг відношення енергій Mark/Space для визначення стану біта.
// Якщо Energy(Mark) > Energy(Space) * ENERGY_RATIO_THRESHOLD, вважаємо Mark.
// Значення > 1.0. Наприклад, 1.5 або 2.0. Потрібно підібрати експериментально.
#define ENERGY_RATIO_THRESHOLD 1.5f

// Максимальний розмір буфера Goertzel. Використовується для статичного виділення пам'яті.
// Має бути >= GOERTZEL_N.
#define GOERTZEL_MAX_N GOERTZEL_N

// --- Визначення станів декодера бітів ---
enum DecoderState {
    IDLE,       // Очікування стартового біта (переходу Mark -> Space тонального детектора)
    START_BIT,  // Обробка стартового біта
    DATA_BITS,  // Обробка 5 бітів даних
    STOP_BIT    // Обробка 1.5 бітного стоп-біта
};

class RTTYDecoder {
   public:
    // Конструктор
    // Параметри конфігурації беруться з #define констант.
    RTTYDecoder();

    // Обробляє один вхідний 8-бітний аудіо семпл.
    // Вхідний семпл очікується як unsigned byte (0-255).
    // Повертає декодований символ, якщо він готовий, інакше 0.
    char process_sample(uint8_t sample);

   private:
    // --- Константні таблиці ITA2 (ITU-2) ---
    // Індексуються 5-бітним кодом (0-31) з LSB у біті 0.
    static const char ITA2_LETTERS[32];
    static const char ITA2_FIGURES[32];

    // --- Розраховані параметри часу бітів ---
    // Кількість семплів на один біт.
    uint16_t samples_per_bit;
    // Кількість семплів на половину біта (для синхронізації по середині старт-біта).
    uint16_t half_bit_samples;
    // Кількість семплів на 1.5 біта (для стоп-біта).
    uint16_t one_half_bit_samples;

    // --- Стан декодера бітів ---
    DecoderState state;  // Поточний стан декодера
    // Кількість семплів з моменту виявлення останнього значущого краю тонального детектора.
    uint16_t samples_since_edge;
    // Кількість семплів з моменту "семплірування" (рішення тонального детектора) середини старт-біта.
    uint16_t samples_since_sync_point;

    uint8_t bit_buffer;     // Буфер для збирання 5 бітів даних
    uint8_t bit_count;      // Лічильник зібраних бітів даних
    bool is_figures_shift;  // Поточний регістр (літери/цифри)

    // --- Стан та буфер тонального детектора (Goertzel) ---
    // Статичний буфер семплів (int16_t для центрованих даних).
    // Використовуємо статичний, щоб уникнути malloc/free на Cortex-M0.
    static int16_t sample_buffer[GOERTZEL_MAX_N];
    uint16_t buffer_idx;  // Поточний індекс запису в циклічний буфер

    // Стан Goertzel для Mark та Space (рекурсивні змінні Q1, Q2).
    // Використовуємо int32_t, оскільки значення можуть зростати.
    int32_t goertzel_mark_Q1;
    int32_t goertzel_mark_Q2;
    int32_t goertzel_space_Q1;
    int32_t goertzel_space_Q2;

    // Коефіцієнти Goertzel у фіксованій точці Q15 (int32_t).
    // Ці константи РОЗРАХОВУЮТЬСЯ ОФЛАЙН для AUDIO_SAMPLE_RATE, GOERTZEL_N, RTTY_F_SPACE, RTTY_F_MARK.
    // Приклад розрахунку для Fs=12000, N=256, f_space=1275, f_mark=1445:
    // k_space = 1275 * 256 / 12000 = 27.2
    // k_mark = 1445 * 256 / 12000 = 30,8266
    // coeff = 2 * cos(2 * PI * k / N) * 2^15
    // coeff_space_Q15 = round(2 * cos(2 * PI * 27.2 / 256) * 32768) = 51467
    // coeff_mark_Q15 = round(2 * cos(2 * PI * 30,8266 / 256) * 32768) = 60964
    static const int32_t coeff_space_Q15 = 51467;  // ПРИКЛАД КОНСТАНТИ
    static const int32_t coeff_mark_Q15 = 65536;   // ПРИКЛАД КОНСТАНТИ

    // Поріг відношення енергій у фіксованій точці Q15.
    // ENERGY_RATIO_THRESHOLD * 2^15. Приклад для 1.5: 1.5 * 32768 = 49152.
    static const int32_t energy_ratio_threshold_Q15 = (int32_t)(ENERGY_RATIO_THRESHOLD * 32768.0f);  // Використовуємо 32768.0f для float

    // Оцінка стану Mark/Space на основі Goertzel.
    bool is_mark_detected;
    bool last_is_mark_detected;  // Попередній стан для виявлення краю

    // Як часто оновлюємо оцінку тону (в семплах). Наприклад, кожні N/4 семплів.
    static const uint16_t tone_update_interval = GOERTZEL_N / 4;
    uint16_t tone_update_counter;  // Лічильник семплів для оновлення тону

    // --- Допоміжні функції ---

    // Виконує один крок Goertzel (для одного семпла).
    // sample: вхідний семпл (центрований int16_t).
    // coeff_Q15: коефіцієнт у Q15.
    // Q1, Q2: поточні стани Goertzel (int32_t), оновлюються за посиланням.
    // Використовує int64_t для проміжного множення, щоб уникнути переповнення.
    void goertzel_step(int32_t coeff_Q15, int16_t sample, int32_t& Q1, int32_t& Q2);

    // Розраховує енергію (квадрат амплітуди) з фінальних Q1, Q2 станів Goertzel.
    // Повертає енергію у форматі int64_t.
    // Використовує int64_t для проміжних обчислень.
    int64_t calculate_goertzel_energy(int32_t coeff_Q15, int32_t Q1, int32_t Q2);

    // Оновлює стан тонального детектора (is_mark_detected).
    // Запускає Goertzel на поточному буфері семплів та порівнює енергії.
    void update_tone_state();

    // Декодує 5-бітний код ITA2 у символ.
    // Повертає символ або 0, якщо це керуючий код (LS/FS).
    char decode_ita2(uint8_t bits);

    // Заборона копіювання та присвоєння
    RTTYDecoder(const RTTYDecoder&) = delete;
    RTTYDecoder& operator=(const RTTYDecoder&) = delete;
};

#endif  // RTTY_AFSK_DECODER_50BAUD_H+