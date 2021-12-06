import sys
reload(sys)
sys.setdefaultencoding('utf-8')


def dec2bin_(dec_num, length):
    """

    :param dec_num:
    :param length:
    :return:
    """
    bin_num = bin(2**length+dec_num)
    bin_num = bin_num.replace('0b1', '')
    return bin_num


def reverse_bin(bin_num):
    """

    :param bin_num:
    :return:
    """
    reverse_bin_num = ""
    for n in bin_num:
        if n == '0':
            reverse_bin_num += '1'
        elif n == '1':
            reverse_bin_num += '0'
    return reverse_bin_num


def complement_bin(bin_num, length):
    """

    :param bin_num:
    :param length:
    :return:
    """
    dec_num = int(bin_num, 2)
    return dec2bin_(dec_num + 1, length)


def dec2bin(dec_num, length):
    """

    :param dec_num:
    :param length:
    :return:
    """
    num_str = str(dec_num)
    num_list = num_str.split('-')
    if len(num_list) == 2:
        bin_num_pre = dec2bin_(-dec_num, length)
        bin_num_pre = reverse_bin(bin_num_pre)
        return complement_bin(bin_num_pre, length)
    elif len(num_list) == 1:
        return dec2bin_(dec_num, length)


def bin2dec(bin_num):
    """

    :param bin_num:
    :return:
    """
    return int(bin_num, 2)


def dec2hex(dec_num):
    """

    :param dec_num:
    :return:
    """
    hex_num = hex(int(dec2bin(dec_num, 8), 2)).replace('0x', '')
    if len(hex_num) == 1:
        hex_num = '0' + hex_num
    return hex_num


def bin2hex(bin_num):
    """

    :param bin_num:
    :return:
    """
    return dec2hex(bin2dec(bin_num))
