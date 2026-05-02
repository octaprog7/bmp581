# micropython
# mail: goctaprog@gmail.com
# MIT license
import micropython
from micropython import const
from collections import namedtuple

from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import IDentifier, IBaseSensorEx, DeviceEx, Iterator, check_value

# ВНИМАНИЕ: не подключайте питание датчика к 5В, иначе датчик выйдет из строя! Только 3.3В!!!
# WARNING: do not connect "+" to 5V or the sensor will be damaged!

_kT = 2 ** -16  # для вычисления температуры
_kP = 2 ** -6  # для вычисления давления

_osr_t_times = 1.0, 1.1, 1.5, 2.1, 3.3, 5.8, 10.8, 20.8
_osr_p_times = 1.0, 1.7, 2.9, 5.4, 10.4, 20.4, 40.4, 80.4


def _get_conv_time_ms(temp_only: bool, osr_t: int, osr_p: int) -> float:
    """Возвращает время преобразования одного значения в [мс].
    Внимание! Это время преобразования, а не частота выходных данных (output data rate) датчика!!!
    Если вы единицу поделите на время_преобразования, это не будет максимальной частотой выдачи данных датчиком!!!"""
    tmp = _osr_t_times[osr_t]
    if temp_only:
        return tmp
    return tmp + _osr_p_times[osr_p]


def _all_none(*args):
    """возвращает Истина, если все входные параметры в None"""
    return all(arg is None for arg in args)


# BMP581 Register Addresses (used regis only)
# Chip ID & Revision
_REG_ADDR_CHIP_ID = const(0x01)      # ASIC identification ID (RO)

# Status & Interrupts
_REG_ADDR_CHIP_STATUS = const(0x11)  # ASIC status register (RO)
_REG_ADDR_INT_CONFIG = const(0x14)   # Interrupt configuration (R/W)
_REG_ADDR_INT_SOURCE = const(0x15)   # INT source selection (R/W)
_REG_ADDR_INT_STATUS = const(0x27)   # Interrupt status, clear-on-read (RO)
_REG_ADDR_STATUS = const(0x28)       # Status register (RO)

# FIFO
_REG_ADDR_FIFO_CONFIG = const(0x16)  # FIFO configuration (R/W)
_REG_ADDR_FIFO_COUNT = const(0x17)   # Number of frames in FIFO (RO)
_REG_ADDR_FIFO_SEL = const(0x18)     # FIFO selection config (R/W)
# _REG_ADDR_FIFO_DATA = const(0x29)    # FIFO output port (RO)

# Data Output Registers (temperature & pressure)
_REG_ADDR_TEMP_XLSB = const(0x1D)    # Temperature XLSB (RO)
_REG_ADDR_TEMP_LSB = const(0x1E)     # Temperature LSB (RO)
_REG_ADDR_TEMP_MSB = const(0x1F)     # Temperature MSB (RO)
_REG_ADDR_PRESS_XLSB = const(0x20)   # Pressure XLSB (RO)
_REG_ADDR_PRESS_LSB = const(0x21)    # Pressure LSB (RO)
_REG_ADDR_PRESS_MSB = const(0x22)    # Pressure MSB (RO)

# DSP & Filter Configuration
_REG_ADDR_DSP_CONFIG = const(0x30)   # DSP configuration (R/W)
_REG_ADDR_DSP_IIR = const(0x31)      # DSP IIR configuration (R/W)

# Oversampling & ODR Configuration
_REG_ADDR_OSR_CONFIG = const(0x36)   # OSR configuration (R/W)
_REG_ADDR_ODR_CONFIG = const(0x37)   # ODR configuration (R/W)
_REG_ADDR_OSR_EFF = const(0x38)      # Effective OSR (RO)

# Command Register
_REG_ADDR_CMD = const(0x7E)          # Command register (W)

# BMP581 Command Codes (CMD register 0x7E)
# =============================================================================
_CMD_SOFT_RESET = const(0xB6)        # Trigger software reset
_CMD_NVM_FIRST = const(0x5D)         # First CMD for NVM read/write sequence
_CMD_NVM_WRITE = const(0xA0)         # Enable NVM programming
_CMD_NVM_READ = const(0xA5)          # Enable NVM read
_CMD_EXTMODE_FIRST = const(0x73)     # Extended mode sequence start
_CMD_EXTMODE_MID = const(0xB4)       # Extended mode sequence middle
_CMD_EXTMODE_LAST = const(0x69)      # Enable extended/debug pages


BMP581_ID = namedtuple("BMP581_ID", "chip_id asic_rev_id")
asic_status = namedtuple("asic_status", "i3c_err_3 i3c_err_0 hif_mode")


class Bmp581(IDentifier, IBaseSensorEx, Iterator):
    """Класс для работы с датчиком давления Bosch BMP581."""

    def __init__(self, adapter: bus_service.BusAdapter, address=0x47):
        """i2c - объект класса I2C; address - адрес датчика на шине"""
        self._connector = DeviceEx(adapter=adapter, address=address, big_byte_order=False)
        self._buf_2 = bytearray(2)  # для _read_buf_from_mem
        self._buf_3 = bytearray(3)  # для _read_buf_from_mem
        # настройки передискретизации температуры
        self._osr_t = 1
        # настройки передискретизации давления
        self._osr_p = 1
        self._mode = 0  # sleep mode
        self._temperature_only = False

    def init_hardware(self):
        """Метод инициализации. Нужно вызывать после конструктора!
        Вынес инициализацию в отдельный метод, для того, чтобы можно было вызвать soft_reset а затем вызвать init!"""
        # Настройка прерываний. Включаю прерывание по готовности данных.
        # Бош расстроили меня! В более старых их датчиках можно
        # было считать флаг готовности данных не включая прерывания !!!
        self._int_source_sel(False, False, True, True)
        # разрешаю прерывания
        self._int_conf(None, True, None, None, None)

    def __del__(self):
        # На момент написания кода, деструкторы в MicroPython не вызываются!
        del self._buf_3
        del self._buf_2

    @property
    def power_mode(self) -> int:
        """текущий режим работы"""
        return self._get_power_mode_or_odr()

    @property
    def temp_oversampling(self) -> int:
        """Возвращает текущие настройки передискретизации температуры из датчика"""
        tmp = self._osr_config()
        return 0x07 & tmp

    @temp_oversampling.setter
    def temp_oversampling(self, value: int):
        r = range(8)
        check_value(value, r, f"Значение вне диапазона: {r.start}..{r.stop - 1}")
        self._osr_t = value

    @property
    def pressure_oversampling(self) -> int:
        """Возвращает текущие настройки передискретизации давления из датчика"""
        tmp = self._osr_config()
        return (0b0011_1000 & tmp) >> 3

    @pressure_oversampling.setter
    def pressure_oversampling(self, value: int):
        r = range(8)
        check_value(value, r, f"Значение вне диапазона: {r.start}..{r.stop - 1}")
        self._osr_p = value

    @property
    def temperature_only(self) -> bool:
        """Если истина, то датчик измеряет только температуру, иначе температуру и давление"""
        return self._temperature_only

    @temperature_only.setter
    def temperature_only(self, value):
        """Если истина, то датчик измеряет только температуру, иначе температуру и давление"""
        self._temperature_only = value

    def _cmd(self, command_code: int):
        """записывает в CMD регистр command_code"""
        check_value(command_code, range(0x100), f"Invalid CMD code: {command_code}")
        # self._write_reg(0x7E, command_code)
        self._connector.write_reg(_REG_ADDR_CMD, command_code, 1)

    def _int_conf(
            self,
            pad_int_drv: int | None = None,  # bit 4..7,
            int_en: bool | None = None,  # bit 3,
            int_od: bool | None = None,  # bit 2,
            int_pol: bool | None = None,  # bit 1,
            int_mode: bool | None = None,  # bit 0,
    ) -> int | None:
        """Interrupt configuration register. Если все параметры в None, возвращает содержимое ICR"""
        val = self._connector.read_reg(_REG_ADDR_INT_CONFIG, 1)[0]
        if _all_none(int_mode, int_pol, int_od, int_en, pad_int_drv):
            return val
        check_value(pad_int_drv, range(0x10), f"Неверное значение pad_int_drv: {pad_int_drv}")
        if pad_int_drv is not None:
            val &= ~0xF0  # mask
            val |= pad_int_drv << 4
        if int_en is not None:
            val &= ~(1 << 3)  # mask
            val |= int_en << 3
        if int_od is not None:
            val &= ~(1 << 2)  # mask
            val |= int_od << 2
        if int_pol is not None:
            val &= ~(1 << 1)  # mask
            val |= int_pol << 1
        if int_mode is not None:
            val &= 0xFE  # mask
            val |= int_mode
        # print(f"DBG:int_conf: {val}")
        self._connector.write_reg(_REG_ADDR_INT_CONFIG, val, 1)
        return None

    def _int_source_sel(
            self,
            oor_p_en: bool | None = None,  # bit 3, Pressure data out-of-range (OOR_P)
            fifo_ths_en: bool | None = None,  # bit 2, FIFO Threshold/Watermark (FIFO_THS)
            fifo_full_en: bool | None = None,  # bit 1, FIFO Full (FIFO_FULL)
            drdy_data_reg_en: bool | None = None,  # bit 0, Data Ready
    ) -> int | None:
        """Interrupt source selection register. Если все параметры в None, возвращает содержимое ISSR"""
        val = self._connector.read_reg(_REG_ADDR_INT_SOURCE, 1)[0]
        if _all_none(oor_p_en, fifo_ths_en, fifo_full_en, drdy_data_reg_en):
            return val
        if oor_p_en is not None:
            val &= ~(1 << 3)  # mask
            val |= oor_p_en << 3
        if fifo_ths_en is not None:
            val &= ~(1 << 2)  # mask
            val |= fifo_ths_en << 2
        if fifo_full_en is not None:
            val &= ~(1 << 1)  # mask
            val |= fifo_full_en << 1
        if drdy_data_reg_en is not None:
            val &= 0xFE  # mask
            val |= drdy_data_reg_en
        # print(f"DBG:int_source_sel: {val}")
        self._connector.write_reg(_REG_ADDR_INT_SOURCE, val, 1)
        return None

    def _fifo_config(
            self,
            fifo_mode: bool | None = None,
            fifo_threshold: int | None = None,
    ) -> int | None:
        """FIFO config. Если все параметры метода None, то возвращает значение регистра!"""
        val = self._connector.read_reg(_REG_ADDR_FIFO_CONFIG, 1)[0]
        if _all_none(fifo_mode, fifo_threshold):
            return val
        check_value(fifo_threshold, range(32), f"Неверное значение fifo_threshold: {fifo_threshold}")
        if fifo_mode is not None:
            val &= ~(1 << 5)
            val |= fifo_mode << 5
        if fifo_threshold is not None:
            val &= ~0x1F
            val |= fifo_threshold
        self._connector.write_reg(_REG_ADDR_FIFO_CONFIG, val, 1)
        return None

    def _odr_config(
            self,
            deep_dis: bool | None = None,      # bit 7. Если Истина, то запрещает глубокий режим ожидания/deep sleep mode (Примечание: это поле нельзя изменить во время текущего преобразования давления/температуры).
            output_data_rate: int | None = None,   # bit 6..2. настроенный ODR может быть недействительным в сочетании с конфигурацией OSR. Это можно увидеть с помощью флага odr_is_valid.
            power_mode: int | None = None,     # bit 1..0. Настройка режима питания. Пользователь может запросить выделенный режим питания, записав это поле. Чтение возвращает фактический режим, в котором находится устройство.
    ) -> int | None:
        """Control 0 Register. Если все параметры метода None, то возвращает значение регистра!
        Величина ODR может быть неверной для текущего значение OSR. Это можно определить с помощью флага
        ODR_CONFIG.flag odr_is_valid. Если настроенные параметры ODR/OSR недействительны, будут использоваться
        настройки OSR по умолчанию."""
        val = self._connector.read_reg(_REG_ADDR_ODR_CONFIG, 1)[0]
        if _all_none(deep_dis, output_data_rate, power_mode):
            return val
        check_value(output_data_rate, range(32), f"Неверное значение ODR: {output_data_rate}")
        check_value(power_mode, range(4), f"Неверное значение power mode: {power_mode}")
        if deep_dis is not None:
            val &= ~(1 << 7)  # mask
            val |= deep_dis << 7
        if output_data_rate is not None:
            val &= ~(0x1F << 2)  # mask
            val |= output_data_rate << 2
        if power_mode is not None:
            val &= ~0b11  # mask
            val |= power_mode
        self._connector.write_reg(_REG_ADDR_ODR_CONFIG, val, 1)
        return None

    def _get_frames_in_fifo(self) -> int:
        """Возвращает кол-во кадров в FIFO"""
        return 0x3F & self._connector.read_reg(_REG_ADDR_FIFO_COUNT, 1)[0]

    def _fifo_sel_config(
            self,
            fifo_dec_sel: int | None = None,
            fifo_frame_sel: int | None = None,
    ) -> int | None:
        """FIFO selection config. Если все параметры метода None, то возвращает значение регистра!"""
        val = self._connector.read_reg(_REG_ADDR_FIFO_SEL, 1)[0]
        if _all_none(fifo_dec_sel, fifo_frame_sel):
            return val
        check_value(fifo_dec_sel, range(8), f"Неверное значение fifo_dec_sel: {fifo_dec_sel}")
        check_value(fifo_frame_sel, range(4), f"Неверное значение fifo_frame_sel: {fifo_frame_sel}")
        if fifo_frame_sel is not None:
            val &= ~0b11
            val |= fifo_frame_sel
        if fifo_dec_sel is not None:
            val &= ~0b111 << 2
            val |= fifo_dec_sel << 2
        self._connector.write_reg(_REG_ADDR_FIFO_SEL, val, 1)
        return None

    def get_id(self) -> BMP581_ID:
        """Возвращает идентификатор датчика и его revision ID.
        Returns the ID and revision ID of the sensor."""
        buf = self._buf_2
        self._connector.read_buf_from_mem(_REG_ADDR_CHIP_ID, buf, 1)
        return BMP581_ID(chip_id=buf[0], asic_rev_id=buf[1])

    def get_status(self, status_source: int = 2) -> None | asic_status | tuple:
        """Возвращает состояние аппаратного модуля датчика.
        id_source == 0 - Возвращает состояние ASIC.
        id_source == 1 - Возвращает состояние Interrupt Status Register (clear-on-read).
        id_source == 2 - Возвращает состояние Status register
        Возвращает кортеж значений битов:
            id == 0:    i3c_err_3, i3c_err_0, hif_mode      три значения
            id == 1:    drdy_data_reg, fifo_full, fifo_ths, oor_p, por      из Interrupt status register. пять значений
            id == 2:    status_core_rdy, status_nvm_rdy, status_nvm_err,
                        status_nvm_cmd_err, status_boot_err_corrected       из Status register. пять значений
        """
        check_value(status_source, range(3), f"Invalid value status_source: {status_source}")
        conn = self._connector
        if 0 == status_source:  # состояние ASIC
            val = conn.read_reg(_REG_ADDR_CHIP_STATUS, 1)[0]
            #   i3c_err_3, i3c_err_0, hif_mode 0..3
            return asic_status(i3c_err_3=0 != (0x08 & val), i3c_err_0=0 != (0x04 & val), hif_mode=0x03 & val)

        if 1 == status_source or 2 == status_source:  # состояние Interrupt Status Register, Status register
            val = conn.read_reg(_REG_ADDR_INT_STATUS if 1 == status_source else _REG_ADDR_STATUS, 1)[0]
            _masks = map(lambda x: 1 << x, range(5))
            # drdy_data_reg, fifo_full, fifo_ths, oor_p, por    from ISR (Interrupt status register)
            # status_core_rdy, status_nvm_rdy, status_nvm_err, ...
            # ... status_nvm_cmd_err, status_boot_err_corrected from Status register
            #
            return tuple([0 != (mask & val) for mask in _masks])
        return None


#    def _get_fifo_output_port(self) -> int:
#        """Возвращает значение регистра FIFO output port"""
#        return self._read_reg(0x29, 1)[0]

    def _dsp_config(self,
                    oor_sel_iir_p: bool | None = None,     # bit 7, Выбор OOR IIR (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    fifo_sel_iir_p: bool | None = None,    # bit 6, FIFO IIR отбор данных давления. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    shdw_sel_iir_p: bool | None = None,    # bit 5, Теневые регистры IIR отбора данных давления. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    fifo_sel_iir_t: bool | None = None,    # bit 4, FIFO IIR отбор данных температуры. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    shdw_sel_iir_t: bool | None = None,    # bit 3, Регистры данных температуры. IIR отбор данных температуры. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    iir_flush_forced_en: bool | None = None,   # bit 2, если в Истина, то IIR фильтр flush выполняется принудительно. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    comp_pt_en: int | None = None,     # bit 1..0, компенсация датчика. bit 0 - компенсация температуры; bit 1 - компенсация давления (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                    ) -> None | int:
        val = self._connector.read_reg(_REG_ADDR_DSP_CONFIG, 1)[0]
        if _all_none(oor_sel_iir_p, fifo_sel_iir_p, shdw_sel_iir_p, fifo_sel_iir_t, shdw_sel_iir_t, iir_flush_forced_en,
                     comp_pt_en):
            return val
        if oor_sel_iir_p is not None:
            val &= ~(1 << 7)  # mask
            val |= oor_sel_iir_p << 7
        if fifo_sel_iir_p is not None:
            val &= ~(1 << 6)  # mask
            val |= fifo_sel_iir_p << 6
        if shdw_sel_iir_p is not None:
            val &= ~(1 << 5)  # mask
            val |= shdw_sel_iir_p << 5
        if fifo_sel_iir_t is not None:
            val &= ~(1 << 4)  # mask
            val |= fifo_sel_iir_t << 4
        if shdw_sel_iir_t is not None:
            val &= ~(1 << 3)  # mask
            val |= shdw_sel_iir_t << 3
        if iir_flush_forced_en is not None:
            val &= ~(1 << 2)  # mask
            val |= iir_flush_forced_en << 2
        if comp_pt_en is not None:
            val &= ~0b11  # mask
            val |= comp_pt_en
        self._connector.write_reg(_REG_ADDR_DSP_CONFIG, val, 1)
        return None

    def _dsp_iir_config(self,
                        set_iir_p: int | None = None,  # bit 5..3, Выбор IIR LPF фильтра давления. Коэффициент фильтра. 0 - фильтр отключен, 0x07 - коэфф. фильтра 127. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                        set_iir_t: int | None = None,  # bit 2..0, Выбор IIR LPF фильтра температуры. Коэффициент фильтра. 0 - фильтр отключен, 0x07 - коэфф. фильтра 127. (Это поле невозможно записать во время текущего преобразования давления/температуры.)
                        ) -> None | int:
        val = self._connector.read_reg(_REG_ADDR_DSP_IIR, 1)[0]
        if _all_none(set_iir_p, set_iir_t):
            return val
        if set_iir_p is not None:
            val &= ~(0b111 << 3)  # mask
            val |= set_iir_p << 3
        if set_iir_t is not None:
            val &= ~0b111  # mask
            val |= set_iir_t
        self._connector.write_reg(_REG_ADDR_DSP_IIR, val, 1)
        return None

    def set_iir_config(self, press_iir: int, temp_iir: int):
        """Производит установку коэффициентов LPF фильтров для давления и температуры.
        press_iir и temp_iir должны быть в диапазоне 0..7 включительно.
        0 - фильтр отключен
        1 - коэфф. фильтра 1
        2 - коэфф. фильтра 3
        3 - коэфф. фильтра 7
        ...
        7 - коэфф. фильтра 127
        Это значения не будут записаны, если датчик производит преобразование сигналов Д/T!"""
        check_value(press_iir, range(8), f"Неверное значение press_iir: {press_iir}")
        check_value(temp_iir, range(8), f"Неверное значение temp_iir: {temp_iir}")
        self._dsp_iir_config(press_iir, temp_iir)

    @property
    def iir_config(self) -> tuple:
        """Возвращает кортеж значений в виде: Коэффициент полосового фильтра LPF для давления,
        Коэффициент полосового фильтра LPF для температуры.
        Коэффициенты в диапазоне 0..7 включительно"""
        tmp = self._dsp_iir_config()
        return (0b0011_1000 & tmp) >> 3, 0b0000_0111 & tmp

    def _osr_config(self,
                    press_en: bool | None = None,  # bit 6, Если Истина, то включается измерение давления датчиком. В противном случае выполняются только измерения температуры.
                    osr_p: int | None = None,  # bit 5..3, частота передискретизации давления (oversampling rate)
                    osr_t: int | None = None,  # bit 2..0, частота передискретизации температуры (oversampling rate)
                    ) -> None | int:
        """Oversampling rates"""
        val = self._connector.read_reg(_REG_ADDR_OSR_CONFIG, 1)[0]
        if _all_none(press_en, osr_p, osr_t):
            return val
        if press_en is not None:
            val &= ~(1 << 6)  # mask
            val |= press_en << 6
        if osr_p is not None:
            val &= ~(0b111 << 3)  # mask
            val |= osr_p << 3
        if osr_t is not None:
            val &= ~0b111  # mask
            val |= osr_t
        self._connector.write_reg(_REG_ADDR_OSR_CONFIG, val, 1)
        return None

    def _eff_osr_config(self) -> tuple:
        """Oversampling rates. Регистр только для чтения!
        Метод возвращает кортеж:    osr_t_eff, osr_p_eff, odr_is_valid
        odr_is_valid: [bool, None] = None,  # bit 7, Если Истина, то параметризация ODR верна!
        (это проверяется при каждом изменении в регистрах конфигурации ODR и OSR).
        osr_p_eff: [int, None] = None,      # bit 5..3, OSR_P выборка. Выборка по давлению. Пожалуйста, обратитесь
        к OSR_CONFIG для значений.
        osr_t_eff: [int, None] = None,      # bit 2..0, OSR_T выборка. Выборка по температуре. Пожалуйста,
        обратитесь к OSR_CONFIG для значений.
        """
        val = self._connector.read_reg(_REG_ADDR_OSR_EFF, 1)[0]
        #       osr_t_eff,      osr_p_eff,              odr_is_valid
        return 0b0111 & val, (0b0011_1000 & val) >> 3, 0 != (0b1000_0000 & val)

    @property
    def effective_osr_rating(self) -> tuple:
        """вычисляемые для текущей ODR значения OSR. Только для чтения.
        Если последний элемент кортежа (odr_is_valid) Истина, то параметризация ODR верна!"""
        return self._eff_osr_config()

    @micropython.native
    def _get_pressure_raw(self) -> int:
        # трех байтовое значение
        buf = self._buf_3
        l, m, h = self._connector.read_buf_from_mem(_REG_ADDR_PRESS_XLSB, buf, 1)
        return (h << 16) | (m << 8) | l

    def get_pressure(self) -> float:
        """Возвращает давление окружающего датчик воздуха в Паскалях [Па]"""
        return _kP * self._get_pressure_raw()

    @micropython.native
    def _get_temperature_raw(self) -> int:
        # трех байтовое значение
        buf = self._buf_3
        # signed 24 bit value
        l, m, h = self._connector.read_buf_from_mem(_REG_ADDR_TEMP_XLSB, buf, 1)
        raw = (h << 16) | (m << 8) | l
        # Конвертация signed 24-bit в Python int
        if raw & 0x800000:  # если бит 23 = 1 → отрицательное
            raw -= 0x1000000
        return raw

    def get_temperature(self) -> float:
        """Возвращает температуру окружающего датчик воздуха в градусах Цельсия"""
        return _kT * self._get_temperature_raw()

    def soft_reset(self):
        """Программный сброс датчика.
        Software reset of the sensor.
        Примечание: при использовании I2C после записи 0xB6 датчик НЕ отправляет ACK!"""
        # После сброса нужно подождать tsoft_res (2 ms) перед следующим чтением из датчика
        # Вызывающий код должен обеспечить задержку или проверку STATUS
        self._connector.write_reg(_REG_ADDR_CMD, _CMD_SOFT_RESET, 1)


    def start_measurement(self, mode: int = 1, output_data_rate: int = 10) -> bool:
        """ # mode:
        0 - Режим ожидания: измерения не производятся.
        1 - Нормальный режим: измерение с частотой output_data_rate.
        2 - Принудительный режим: принудительное однократное(!) измерение.
        3 - Непрерывный режим: измерения с наибольшей частотой при текущих настройках.
        Возвращает бит odr_is_valid регистра OSR_EFF для проверки правильности output_data_rate в комбинации с
        temperature oversampling и pressure oversampling"""
        if mode not in range(4):
            raise ValueError(f"Неверное значение режима работы: {mode}")
        _osr_t = self.temp_oversampling
        _osr_p = self.pressure_oversampling
        press_enable = not self.temperature_only
        self._osr_config(press_enable, _osr_p, _osr_t)
        self._odr_config(True, output_data_rate, mode)
        self._mode = mode
        # чтобы можно было точнее(!) вычислить время преобразования, обновляю поля класса
        self._osr_t, self._osr_p, odr_is_valid = self._eff_osr_config()
        # возвращаю поле odr_is_valid регистра OSR_EFF
        return odr_is_valid

    def _get_power_mode_or_odr(self, odr: bool = False) -> int:
        """Возвращает текущий режим работы датчика или output data rate:
        0 - Режим ожидания: измерения не проводятся.
        1 - Нормальный режим: измерение с частотой ODR.
        2 - Принудительный режим: принудительное однократное измерение
        3 - Непрерывный режим: повторяющиеся измерения."""
        tmp = self._odr_config()
        if odr:
            return (0b0111_1100 & tmp) >> 2
        return 0b0000_0011 & tmp

    @property
    def output_data_rate(self) -> int:
        return self._get_power_mode_or_odr(True)

    def get_oversampling(self) -> tuple:
        """возвращает кортеж вида pressure_oversampling: int, temperature_oversampling"""
        tmp = self._osr_config()
        return (tmp & 0b0011_1000) >> 3, tmp & 0b0000_0111

    @property
    def oversampling(self) -> tuple:
        return self.get_oversampling()

    def is_data_ready(self) -> bool:
        """Возвращает Истина, когда есть новые данные температуры или давления для считывания"""
        tmp = self.get_status(1)
        # print(f"DBG:is_data_ready: {tmp}")
        return tmp[0]

    @micropython.native
    def get_conversion_cycle_time(self) -> float:
        """возвращает время преобразования в [мс] датчиком температуры или давления в зависимости от его настроек"""
        return _get_conv_time_ms(self.temperature_only, self.temp_oversampling, self.pressure_oversampling)

    # Iterator
    def __next__(self) -> tuple | float | None:
        if not self.is_data_ready():
            return None  # No data!
        temperature = self.get_temperature()
        if self.temperature_only:
            return temperature
        return self.get_pressure(), temperature
