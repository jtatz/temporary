import serial

class Dmm:
    bytesPerRead = 14

    def __init__(self, port='/dev/ttyUSB0', retries=3, timeout=3.0):
        self.ser = serial.Serial(
            port=port,
            baudrate=2400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
        self.retries = retries
        self._synchronize()

    def close(self):
        "Close the serial port connection."
        self.ser.close()

    def read(self):
        "Attempt to take a reading from the digital multimeter."
        success = False
        for readAttempt in range(self.retries):
            bytes = self.ser.read(self.bytesPerRead)
            if len(bytes) != self.bytesPerRead:
                self._synchronize()
                continue

            for pos, byte in enumerate(bytes, start=1):
                if byte // 16 != pos:
                    self._synchronize()
                    break
            else:
                success = True
                break

            # if we're here we need to resync and retry
            self._synchronize()

        if not success:
            raise DmmReadFailure()

        val = ''
        for (d1, d2, ch) in self.digits:
            highBit, digit = self._readDigit(bytes[d1 - 1], bytes[d2 - 1])
            if highBit:
                val += ch
            val += digit

        attribs = self._initAttribs()
        for k, v in self.bits.items():
            self._readAttribByte(bytes[k - 1], v, attribs)

        return DmmValue(val, attribs, readAttempt, bytes)

    def _synchronize(self):
        v = self.ser.read(1)
        if len(v) != 1:
            raise DmmNoData()
        n = v[0]
        pos = n // 16
        if pos == 0 or pos == 15:
            raise DmmInvalidSyncValue()

        bytesNeeded = self.bytesPerRead - pos
        if bytesNeeded:
            v = self.ser.read(bytesNeeded)

    bits = {
        1: [('flags', 'AC'), ('flags', 'DC'), ('flags', 'AUTO'), ('flags', 'RS232')],
        10: [('scale', 'micro'), ('scale', 'nano'), ('scale', 'kilo'), ('measure', 'diode')],
        11: [('scale', 'milli'), ('measure', '% (duty-cycle)'), ('scale', 'mega'),
             ('flags', 'beep')],
        12: [('measure', 'Farads'), ('measure', 'Ohms'), ('flags', 'REL delta'),
             ('flags', 'Hold')],
        13: [('measure', 'Amps'), ('measure', 'volts'), ('measure', 'Hertz'),
             ('other', 'other_13_1')],
        14: [('other', 'other_14_4'), ('measure', 'degrees Celcius'), ('other', 'other_14_2'),
             ('other', 'other_14_1')]
    }

    digits = [(2, 3, '-'), (4, 5, '.'), (6, 7, '.'), (8, 9, '.')]

    digitTable = {(0, 5): '1', (5, 11): '2', (1, 15): '3', (2, 7): '4', (3, 14): '5',
                  (7, 14): '6', (1, 5): '7', (7, 15): '8', (3, 15): '9', (7, 13): '0',
                  (6, 8): 'L', (0, 0): ' '}

    def _initAttribs(self):
        return {'flags': [], 'scale': [], 'measure': [], 'other': []}

    def _readAttribByte(self, byte, bits, attribs):
        b = byte % 16
        bitVal = 8
        for (attr, val) in bits:
            v = b // bitVal
            if v:
                b -= bitVal
                attribs[attr].append(val)
            bitVal //= 2

    def _readDigit(self, byte1, byte2):
        b1 = byte1 % 16
        highBit = b1 // 8
        b1 %= 8
        b2 = byte2 % 16
        try:
            digit = self.digitTable[(b1, b2)]
        except KeyError:
            digit = 'X'
        return highBit, digit


class DmmValue:
    def __init__(self, val, attribs, readErrors, rawBytes):
        self.saneValue = True
        self.rawVal = self.val = val
        self.flags = attribs['flags']
        self.scaleFlags = attribs['scale']
        self.measurementFlags = attribs['measure']
        self.reservedFlags = attribs['other']
        self.readErrors = readErrors
        self.rawBytes = rawBytes
        self.text = 'Invalid Value'

        self.processFlags()
        self.processScale()
        self.processMeasurement()
        self.processVal()

        if self.saneValue:
            self.createTextExpression()

    def createTextExpression(self):
        text = self.deltaText
        text += self.val
        text += ' '
        text += self.scale
        text += self.measurement
        text += self.ACDCText
        self.text = text

    def processFlags(self):
        flags = self.flags
        self.ACDC = None
        self.ACDCText = ''
        self.delta = False
        self.deltaText = ''

        if 'AC' in flags and 'DC' in flags:
            self.saneValue = False
        if 'AC' in flags:
            self.ACDC = 'AC'
        if 'DC' in flags:
            self.ACDC = 'DC'
        if self.ACDC is not None:
            self.ACDCText = ' ' + self.ACDC
        if 'REL delta' in flags:
            self.delta = True
            self.deltaText = 'delta '

    scaleTable = {'nano': 0.000000001, 'micro': 0.000001, 'milli': 0.001,
                  'kilo': 1000.0, 'mega': 1000000.0}

    def processScale(self):
        s = self.scaleFlags
        self.scale = ''
        self.multiplier = 1

        if not s:
            return
        if len(s) > 1:
            self.saneValue = False
            return
        self.scale = s[0]
        self.multiplier = self.scaleTable[self.scale]

    def processMeasurement(self):
        m = self.measurementFlags
        self.measurement = None
        if len(m) != 1:
            self.saneValue = False
            return
        self.measurement = m[0]

    def processVal(self):
        v = self.rawVal
        self.numericVal = None
        if 'X' in v:
            self.saneValue = False
            return
        if v.count('.') > 1:
            self.saneValue = False
            return

        n = None
        try:
            n = float(v)
        except ValueError:
            pass

        if n is not None:
            self.val = '%s' % n  # this should remove leading zeros, spaces etc.
            self.numericVal = n * self.multiplier

    def __repr__(self):
        return f"<DmmValue instance: {self.text}>"


class DmmException(Exception):
    "Base exception class for Dmm."


class DmmNoData(DmmException):
    "Read from serial port timed out with no bytes read."


class DmmInvalidSyncValue(DmmException):
    "Got an invalid byte during synchronization."


class DmmReadFailure(DmmException):
    "Unable to get a successful read within the number of allowed retries."


def main():
    dmm = Dmm()

    while True:
        val = dmm.read()
        print(val.text)
        print(val.numericVal)


# main hook
if __name__ == "__main__":
    main()
