import re
from scale_trans import *


def compute_signal_value(physical_value, accuracy, bias):
    """

    :param physical_value:
    :param accuracy:
    :param bias:
    :return:
    """
    return int((physical_value - bias) / accuracy)


def write_bit_intel(bin_data_list, bin_s_data_list, position):
    for i in range(len(bin_s_data_list)):
        bin_data_list[position+i] = bin_s_data_list[-i-1]
    return bin_data_list


def write_bit_motorola(bin_data_list, bin_s_data_list, position):
    bin_data_list[position:position+len(bin_s_data_list)] = bin_data_list
    return bin_data_list


class Messages:
    def __init__(self):
        self.messages = {}

    def add_message(self, message):
        self.messages[message.message_name] = message

    def get_message_by_name(self, message_name):
        return self.messages[message_name]


class Message:
    def __init__(self, frame_id, message_name, frame_length, node, signals=None):
        self.frame_id = frame_id
        self.message_name = message_name
        self.frame_length = frame_length
        self.node = node
        self.signals = signals

    def add_signals(self, signals):
        self.signals = dict()
        for i in range(len(signals)):
            self.signals[signals[i].signal_name] = signals[i]

    def encode(self, physical_values):
        """
        {'singal_name_1": phy_values_1, ...}
        :param physical_values:
        :type physical_values: dict
        :return:
        """
        keys = physical_values.keys()
        bin_data_list = list(64*'0')
        for key in keys:
            mode = self.signals[key].mode
            accuracy = self.signals[key].accuracy
            bias = self.signals[key].bias
            position = self.signals[key].position
            length = self.signals[key].length
            signal_value = compute_signal_value(physical_values[key], accuracy, bias)
            if mode == '1':
                bin_data_list = write_bit_intel(bin_data_list, list(dec2bin(signal_value, length)), position)
            elif mode == '0':
                bin_data_list = write_bit_motorola(bin_data_list, list(dec2bin(signal_value, length)), position)

        # hex_data = ""
        data_array = []
        for i in range(8):
            bin_data = bin_data_list[i*8:i*8+8]
            if mode == '1':
                bin_data.reverse()
            # bin_data = "".join(bin_data)
            # hex_data += scale_trans.bin2hex(bin_data)
            data_array.append(bin2dec("".join(bin_data)))
        # hex_data = bytes(bytearray.fromhex(hex_data))
        # return hex_data
        return data_array

    def decode(self, data_array):
        """
        decode a dec canbus signal array to physical values
        @param data_array: dec signal array
        @return:
        """
        bin_str = ""
        for i in range(len(data_array)):
            bin_str += dec2bin(data_array[i], 8)[::-1]
        decoded_value = dict()
        for key in self.signals.keys():
            signal = self.signals[key]
            accuracy = signal.accuracy
            bias = signal.bias
            position = signal.position
            length = signal.length
            low = signal.low
            high = signal.high
            if int(signal.mode) == 1:
                signal_value = bin_str[position:position+length][::-1]
            else:
                signal_value = bin_str[position:position+length]
            physical_value = bin2dec(signal_value)
            if ((physical_value < low) | (physical_value > high)) & (low != high):
                physical_value = physical_value - 1
                physical_value = -bin2dec(reverse_bin(dec2bin_(physical_value, 16)))
                physical_value = int(physical_value * accuracy + bias)
            else:
                physical_value = int(physical_value * accuracy + bias)
            decoded_value[key] = physical_value
        return decoded_value

    def __del__(self):
        pass


class Signal:
    def __init__(self, signal_name, position, length, mode, is_signed, accuracy, bias, low, high, unit, node):
        self.signal_name = signal_name
        self.position = position
        self.length = length
        self.mode = mode
        self.is_signed = is_signed
        self.accuracy = accuracy
        self.bias = bias
        self.low = low
        self.high = high
        self.unit = unit
        self.node = node

    def __del__(self):
        pass


def decode_message(message_str):
    """
    parse message from dbc message line

    :param message_str:
    :return:
    """
    str_list = message_str.split()
    message = Message(
        frame_id=int(str_list[1]),
        message_name=str_list[2].replace(":", ""),
        frame_length=int(str_list[3]),
        node=str_list[4]
    )
    return message


def decode_signal(signal_str):
    """
    parse signal from dbc signal line

    :param signal_str:
    :return:

    """
    str_list = signal_str.split()
    signal_name = str_list[1]
    p_m = str_list[3].split('@')
    position = p_m[0]
    p = position.split('|')
    position = int(p[0])
    length = int(p[1])
    mode = p_m[1][0]
    if p_m[1][1] == '-':
        is_signed = True
    elif p_m[1][1] == '+':
        is_signed = False
    accuracy = float(str_list[4][1])
    bias = float(str_list[4][3])
    low_high = str_list[5].split('|')
    low = float(low_high[0][1:])
    high = float(low_high[1][:-1])
    unit = str_list[6].replace('"', "")
    node = str_list[7]
    signal = Signal(
        signal_name=signal_name,
        position=position,
        length=length,
        mode=mode,
        is_signed=is_signed,
        accuracy=accuracy,
        bias=bias,
        low=low,
        high=high,
        unit=unit,
        node=node
    )
    return signal


def decode_dbc(file_path):
    dbc_file = open(file_path, 'r')
    line_len = dbc_file.__sizeof__()
    messages = Messages()
    signals = []
    for i in range(line_len):
        line = dbc_file.readline()
        mo = re.match(r'(BO_) (\d*)', line)
        if mo:
            message = decode_message(line)
            for j in range(20):
                line = dbc_file.readline()
                if line != "\r\n":
                    signal = decode_signal(line)
                    signals.append(signal)
                    i += 1
                else:
                    message.add_signals(signals)
                    messages.add_message(message)
                    signals = []
                    break
        else:
            pass
    return messages


if __name__ == '__main__':
    ms = decode_dbc('/home/ahua/pix.dbc')
    m = ms.get_message_by_name('Auto_control')
    code = m.encode({'Steering':500, 'self_driving_enable':1})
    print(code)
