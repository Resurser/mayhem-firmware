#ifndef __RTTY_DECODER_H__
#define __RTTY_DECODER_H__

#include <cstdint> // Для uint8_t, uint16_t, int16_t, int32_t, int64_t
// #include <cmath> // Може знадобитися для розрахунку коефіцієнтів один раз під час ініціалізації
// #define _USE_MATH_DEFINES // Для M_PI у Visual Studio (якщо використовуєте cmath)

// Визначення станів декодера бітів
enum DecoderState {
    IDLE,         // Очікування стартового біта (переходу Mark -> Space тонального детектора)
    START_BIT,    // Обробка стартового біта
    DATA_BITS,    // Обробка 5 бітів даних
    STOP_BIT      // Обробка 1.5 бітного стоп-біта
};

// Максимальний розмір буфера Goertzel. Необхідно для статичного буфера.
// Оберіть достатній розмір, наприклад, 512 або 1024, залежно від вимог до Fs та N.
#define GOERTZEL_MAX_N 512

class RTTYDecoder {
public:
    // Конструктор
    // sample_rate: частота дискретизації вхідних аудіосемплів (семплів/сек). З файлу!
    // baud_rate: швидкість передачі RTTY (бод). 50 для цього випадку.
    // f_space: частота Space тону. 1275 Гц для цього випадку.
    // f_mark: частота Mark тону. 1445 Гц для цього випадку.
    // goertzel_n: розмір блоку Goertzel. Наприклад, 256.
    // energy_ratio_threshold: поріг відношення енергій (наприклад, 1.5). Mark, якщо E_mark > E_space * threshold
    RTTYDecoder(uint32_t sample_rate, uint16_t baud_rate, uint16_t f_space, uint16_t f_mark, uint16_t goertzel_n, float energy_ratio_threshold);

    // TODO: Якщо використовуєте динамічний буфер, додати деструктор для звільнення пам'яті
    // ~RTTYDecoder();

    // Обробляє один вхідний 8-бітний аудіо семпл (беззнаковий 0-255).
    // Повертає декодований символ (як char), якщо він готовий, інакше 0.
    char process_sample(uint8_t sample);

private:
    // Константні таблиці ITA2 (ITU-2)
    static const char ITA2_LETTERS[32];
    static const char ITA2_FIGURES[32];

    // -- Параметри --
    uint32_t sample_rate;
    uint16_t baud_rate;
    uint16_t f_space;
    uint16_t f_mark;
    uint16_t goertzel_n;
    // Поріг відношення енергій у фіксованій точці Q15
    int32_t energy_ratio_threshold_Q15;

    // -- Розраховані параметри часу бітів --
    uint16_t samples_per_bit;      // Кількість семплів на один біт
    uint16_t half_bit_samples;     // Кількість семплів на половину біта
    uint16_t one_half_bit_samples; // Кількість семплів на 1.5 біта (для стоп-біта)

    // -- Стан декодера бітів --
    DecoderState state;             // Поточний стан декодера
    uint16_t samples_since_edge;    // Кількість семплів з моменту виявлення останнього краю тонального детектора
    uint16_t samples_since_sync_point; // Кількість семплів з моменту "семплірування" (рішення тонального детектора) середини старт-біта

    uint8_t bit_buffer;             // Буфер для збирання 5 бітів даних
    uint8_t bit_count;              // Лічильник зібраних бітів даних
    bool is_figures_shift;          // Поточний регістр (літери/цифри)

    // -- Стан та буфер тонального детектора (Goertzel) --
    // Використовуємо статичний буфер для Cortex-M0
    int16_t sample_buffer[GOERTZEL_MAX_N]; // Буфер семплів (int16_t для центрованих даних -128..127)
    uint16_t buffer_idx;           // Поточний індекс запису в буфер (куди писати наступний семпл)

    // Стан Goertzel для Mark та Space
    int32_t goertzel_mark_Q1;
    int32_t goertzel_mark_Q2;
    int32_t goertzel_space_Q1;
    int32_t goertzel_space_Q2;

    // Коефіцієнти Goertzel у фіксованій точці Q15 (2 * cos(2*PI*k/N) * 2^15)
    // ЦІ ЗНАЧЕННЯ ПОВИННІ БУТИ РОЗРАХОВАНІ ТОЧНО ДЛЯ ВАШИХ Fs, N, f_space, f_mark
    int32_t coeff_mark_Q15;
    int32_t coeff_space_Q15;

    // Оцінка стану Mark/Space на основі Goertzel
    bool is_mark_detected;
    bool last_is_mark_detected; // Попередній стан для виявлення краю

    uint16_t tone_update_interval; // Як часто оновлюємо оцінку тону (в семплах). Наприклад, N/4
    uint16_t tone_update_counter;  // Лічильник семплів для оновлення тону

    // -- Допоміжні функції --

    // Виконує один крок Goertzel рекурсії.
    // coeff_Q15: коефіцієнт 2*cos(...) у Q15.
    // sample: вхідний семпл (центрований int16_t).
    // Q1, Q2: поточні стани Goertzel (int32_t), оновлюються за посиланням.
    void goertzel_step(int32_t coeff_Q15, int16_t sample, int32_t& Q1, int32_t& Q2);

    // Розраховує енергію з фінальних Q1, Q2 станів Goertzel.
    // coeff_Q15: коефіцієнт 2*cos(...) у Q15.
    // Повертає енергію у форматі int64_t (квадрат амплітуди).
    int64_t calculate_goertzel_energy(int32_t coeff_Q15, int32_t Q1, int32_t Q2);

    // Оновлює стан тонального детектора (is_mark_detected).
    // Запускає Goertzel на поточному буфері семплів.
    void update_tone_state();

    // Декодує 5-бітний код ITA2.
    // Повертає символ або 0, якщо це керуючий код (LS/FS).
    char decode_ita2(uint8_t bits);

    // Заборона копіювання та присвоєння
    RTTYDecoder(const RTTYDecoder&) = delete;
    RTTYDecoder& operator=(const RTTYDecoder&) = delete;
};

#endif // __RTTY_DECODER_H__