# Интерфейсы первой реализации

Структура: Arduino ESP32 3.3.12; общий код C++11 без Arduino-зависимостей в `libraries/SorterCore/src`; прошивки в `firmware`; инструменты Python в `tools`. Основная камера пока только профиль AI-Thinker classic ESP32-CAM, явно неподтверждённый пользователем. Сервоприводные выходы и межплатный UART не активируются без разрешения локальной конфигурации; диагностическая USB-консоль включена. Никакого подключения/прошивки физических устройств автоматически.

## UART v1

ASCII кадр `@1|TYPE|SESSION|SEQ|PAYLOAD*CCCC\n`. CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, xorout0) по ASCII между `@` и `*`, не включая их. CRC четыре hex символа. SESSION и SEQ uint32 decimal, ненулевые. Максимум 480 байт тела, 512 байт буфер на кадр. В payload запрещены `@`, `|`, `*`, CR и LF. Типы Q (запрос снимка), D (результат), E (ошибка). Отладочные сообщения только через USB Serial, UART1 отдельный.

Контроллер движения создаёт SESSION через esp_random(), ненулевой, и SEQ для каждого нового снимка. Запрос Q payload `calibration_id` (uint32 nonzero). Камера отвечает D с теми же SESSION/SEQ; payload `camera_boot,calibration_id,count;class_id,x10,y10,pixels;...`. camera_boot ненулевой uint32, count0..8, class1..3, x10/y10 int32 в десятых миллиметра, pixels uint32 >0. Нет ориентации: первая версия для круглых деталей. E payload короткий ASCII код причины.

Все числовые поля строго разобраны: лишние символы/поля, переполнение, invalid count/CRC отвергаются. Камера хранит ответ на последний запрос (SESSION,SEQ,calibration_id). Дубликат возвращает тот же ответ, не делает новый снимок. Тот же SESSION/SEQ с другим calibration_id → E CONFLICT. При изменении SESSION сбросить кэш; это новая сессия контроллера. Старый запрос с меньшим SEQ в текущей сессии отвергнуть (wrap sequence означает новую session).

Управляющая плата принимает только текущие SESSION/SEQ и calibration_id, только в состоянии ожидания снимка. Результат потребляется единожды. При повторе Q используется прежний SEQ, после двух повторов FAULT. Новый снимок только после возвращения в парковку и выдержки. Изменение camera_boot в действующей рабочей сессии → FAULT. Первый ответ задаёт ожидаемый camera_boot. Ответы при отключённом управлении не запускают движения.

## Общая библиотека Protocol.h

namespace sorter: `constexpr size_t kMaxPayload=400`, `kMaxBody=480`, `kMaxFrame=512`, `kMaxObjects=8`; `struct Packet {char type; uint32_t session,seq; char payload[kMaxPayload+1];}`; `uint16_t crc16(const uint8_t*,size_t)`; `bool encodePacket(const Packet&,char*,size_t)`; `bool decodePacket(const char*,Packet&)`; `class LineParser { public: bool feed(char,Packet&); void reset(); uint32_t errors() const; }`.

`struct Detection {uint8_t classId; int32_t x10,y10; uint32_t pixels;}`; `struct Scene {uint32_t cameraBoot,calibrationId; uint8_t count; Detection objects[kMaxObjects];}`; `bool encodeScene(const Scene&,char*,size_t)`; `bool decodeScene(const char*,Scene&)`; `bool parseU32(const char*,uint32_t&)`.

Обе прошивки используют эту библиотеку; её же функции проверяются нативными тестами на компьютере.

## Конфигурация и ограничения

Разрешение камеры 320×240 (сначала диагностика160×120). Пример матрицы гомографии/цветовых порогов НЕ является измеренной калибровкой: флаг calibration_confirmed=false. Матрицу калибровки записывает PC инструмент, cal_id0 означает невалидную. Выводы UART и PWM согласованы с текущим файлом KiCad, но монтаж не проверен; отсутствующие входы разрешения и контроля питания описаны в docs/wiring.md. Камера включается только после CAMERA_PROFILE_CONFIRMED=true. Движение аппаратно не включается при MOTOR_OUTPUTS_ENABLED=false; по умолчанию виртуальный прогон, плюс проверяемые флаги геометрии/пинов/нулей. Нужен отдельный физический вход разрешения/отключения с default disabled и явная команда ARM через локальную USB консоль, новые ARM после power loss. Ручные диагностические команды ограничены безопасной конфигурацией.

Аппаратные испытания не проводились: проверка компиляции не подтверждает работу механики, калибровки и отключения питания. Не подменять это утверждением готового работающего робота.
