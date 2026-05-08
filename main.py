# This is a sample Python script.
import bmp581mod
from time import sleep_ms
from machine import I2C, Pin
from micropython import const
from sensor_pack_2.bus_service import I2cAdapter

txt_break = 32 * "-"

I2C_ID: int = const(1)
SCL_PIN: int = const(7)
SDA_PIN: int = const(6)
I2C_FREQ: int = const(400_000)
SENSOR_ADDR: int = const(0x47)
ITERATIONS: int = const(33)


def get_wait_time_ms(output_data_rate: int, conv_time_ms: float = 0) -> int:
    """Возвращает период(!) выдачи данных [мс] с учётом времени конвертации"""
    if output_data_rate < 1 or output_data_rate > 400:
        raise ValueError(f"Частота выдачи данных неверна!")
    # Базовый период + запас на конвертацию
    return 1 + max(int(1000 / output_data_rate), int(conv_time_ms))


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе I2C, для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # i2c = I2C(id=0, scl=Pin(13), sda=Pin(12), freq=400_000)  # on Arduino Nano RP2040 Connect
    i2c = I2C(id=I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)   # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)
    # ps - pressure sensor
    ps = bmp581mod.Bmp581(adapter=adapter, address=SENSOR_ADDR)
    # программный сброс датчика
    ps.soft_reset()
    sleep_ms(2)    # продолжительность программного сброса не более 2 мс!
    #
    ps.init_hardware()
    # если у вас посыпались исключения, то проверьте все соединения!
    res = ps.get_id()
    print(res)
    status = ps.get_status(2)    # запрос состояния из регистра состояния
    status_core_rdy, status_nvm_rdy, status_nvm_err, status_nvm_cmd_err, status_boot_err_corrected = status
    print(f"Регистр состояния: core_rdy: {status_core_rdy}, nvm_rdy: {status_nvm_rdy}, nvm_error: {status_nvm_err}, nvm_cmd_error: {status_nvm_cmd_err}")
    status = ps.get_status(1)  # запрос состояния Interrupt status register
    drdy_data_reg, fifo_full, fifo_ths, oor_p, por = status
    print(f"Состояние ISR: data_rdy: {drdy_data_reg}, FIFO full: {fifo_full}, FIFO Threshold: {fifo_ths}, Pressure data out of range: {oor_p}, POR or software reset complete: {por}")
    status_ok = status_nvm_rdy and not status_nvm_err and por
    if status_ok:
        print("Состояние датчика в норме!")
    else:
        print("Состояние датчика неверное. Возможна неисправность!")

    print(txt_break)
    print("Текущие настройки, считанные из датчика.")
    _oversampling = ps.set_oversampling(None, None)
    _pwr_mode = ps.set_power_mode(None)
    _output_data_rate = ps.set_sampling_period(None)
    iir_conf = ps.set_iir_filter(None, None)         # (temp_iir, press_iir)

    print(f"power mode: {_pwr_mode}; output data rate: {_output_data_rate}")
    print(f"pressure oversampling: {_oversampling[0]}; temperature oversampling: {_oversampling[1]}")
    print(f"pressure iir: {iir_conf[1]}; temperature iir: {iir_conf[0]}")
    print(txt_break)

    osr_r = ps.get_effective_osr_rating()
    print(f"OSR Tэфф: {osr_r[0]}; OSR Pэфф: {osr_r[1]}; Настройка ODR верна: {osr_r[2]}")

    print(txt_break)
    print("Измеряется только температура!")
    ps.set_channels(temp_en=True, press_en=False)  # Только температура
    ps.set_oversampling(temp=4, press=4)  # press игнорируется при press_en=False
    odr = 10    # индекс(!) частоты выдачи данных
    conv_time = ps.get_conversion_cycle_time()  # возвращает мс
    wt_ms = get_wait_time_ms(odr, conv_time)
    print(f"Период выдачи данных [мс]: {wt_ms}")
    ps.set_power_mode(value=1)  # Normal
    ps.set_sampling_period(odr)
    tmp = ps.start_measurement()
    if tmp:
        print("Частота обновления данных датчиком соответствует значениям передискретизации температуры и давления!")

    for _ in range(ITERATIONS):
        sleep_ms(wt_ms)
        if ps.is_data_ready():
            temperature = ps.get_temperature()
            print(f"Температура [гр. Ц.]: {temperature}")
        else:
            print(f"Нет данных для считывания!")
    print(txt_break)
    print("Измеряется температура и давление!")
    ps.set_channels(temp_en=True, press_en=True)  # Температура + давление
    odr = 20    # индекс(!) частоты выдачи данных
    conv_time = ps.get_conversion_cycle_time()  # возвращает мс
    wt_ms = get_wait_time_ms(odr, conv_time)
    ps.set_power_mode(value=1)
    ps.set_sampling_period(odr)
    tmp = ps.start_measurement()
    for _ in range(ITERATIONS):
        sleep_ms(wt_ms)
        if ps.is_data_ready():
            temperature, pressure = ps.get_temperature(), ps.get_pressure()
            print(f"Температура [гр. Ц.]: {temperature}; Давление [Па]: {pressure}")
        else:
            print(f"Нет данных для считывания!")

    print(txt_break)
    odr = 5     # индекс(!) частоты выдачи данных
    conv_time = ps.get_conversion_cycle_time()  # возвращает мс
    wt_ms = get_wait_time_ms(odr, conv_time)
    print("Режим измерения по запросу! Температура и давление!")
    print("Применение протокола итератора.")
    # Режим измерения по запросу рекомендуется для приложений, которым требуется очень низкая частота дискретизации или
    # синхронизация на базе хоста. Режим измерения по запросу также можно использовать, если требуется ODR выше 240 Гц.
    # задержка перед первым next(), чтобы датчик успел подготовить данные
    ps.temperature_only = False
    ps.set_power_mode(value=2)  # Forced
    ps.set_sampling_period(0)  # ODR игнорируется в Forced mode

    for _ in range(ITERATIONS):
        ps.start_measurement()  # Запускаю измерение
        min_conv = ps.get_conversion_cycle_time()
        sleep_ms(max(wt_ms, int(min_conv) + 2))    # Жду

        if ps.is_data_ready():
            # Читаю напрямую, без итератора!
            t = ps.get_temperature()
            p = ps.get_pressure()
            print(f"Температура [гр. Ц.]: {t}; Давление [Па]: {p}")
        else:
            print("Данные не готовы!")
    print(txt_break)
