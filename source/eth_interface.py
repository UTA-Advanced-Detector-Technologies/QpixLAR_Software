#!/usr/bin/env python

import struct
import datetime

# Qt5 dependencies
from PyQt5.QtCore import QObject, QByteArray, pyqtSignal, QThread
from PyQt5.QtNetwork import QTcpSocket, QHostAddress, QUdpSocket

# global defualts to configure connection to socket
ETH_IP      = '192.168.1.10' # set local ethernet port to 192.169.1.17 to connect
ETH_PORT    = 7
BUFFER_SIZE = 1024

# UDP Info
ETH_UDP_IP   = '192.168.1.10'
ETH_UDP_PORT = 8
EXIT_PACKET = bytes("ZaiJian", encoding="utf-8")
PACKET_HEADER = bytes("HEADER", encoding="utf-8")
DEFAULT_PACKET_SIZE = 1
LATEST_STABLE_VERSION = 0x0000_00001


class QDBBadAddr(Exception):
    pass


class EthBadAddr(Exception):
    pass


class UDPworker(QObject):
    """
    This class is sent to a new thread, and monitors
    the output of the UDP port that sends burst data.
    It should listen to and read from the UDP socket, and then dump
    all data into an output file.
    """
    finished = pyqtSignal()
    new_data = pyqtSignal(object)

    def __init__(self):
        super().__init__()

        # create and manage the new thread once running
        self._udpsocket = QUdpSocket(self)
        self._stopped = True
        self.output_file = None

        self._first = 0

    def _udp_connect(self):
        # try to connect to the UDP socket
        print("udp connecting..", end="")
        connected = False
        try:
            bound = self._udpsocket.bind(QHostAddress(ETH_UDP_IP), ETH_UDP_PORT)
            print("Thread UDP..", end=" ")
            if bound:
                print("connected!")
                connected = True
            else:
                print("ERROR!! UDP unconnected!..")

        except Exception as ex:
            print(ex)

        return connected

    def on_readyRead(self):
        while self._udpsocket.hasPendingDatagrams():
            datagram = QByteArray()
            datagram.resize(self._udpsocket.pendingDatagramSize())
            datagram, host, port = self._udpsocket.readDatagram(self._udpsocket.pendingDatagramSize())
            if datagram == EXIT_PACKET:
                self.f.close()
                self.finished.emit()
                self._udpsocket.close()
                return
            else:
                self.new_data.emit(datagram)
                size = len(datagram)
                nresets = int((size-2)/8)
                self.f.write(PACKET_HEADER+size.to_bytes(4, byteorder="little")+datagram)

    def run(self):
        if self._first == 0:
            self._udpsocket.readyRead.connect(self.on_readyRead)
            self._first += 1
        if not self._udp_connect():
            self.finished.emit()
        else:
            self.output_file = datetime.datetime.now().strftime('./bin/%m_%d_%Y_%H_%M_%S.bin')
            self.f = open(self.output_file, 'wb')


class eth_interface(QObject):
    """
    Generic interface class which manages the socket transactions and retrieves
    data from transactions.

    This class is responsible for handling signals and slots between the
    tcpsocket and the Zybo.

    The only public methods that should be used from this interface are reading
    and writing between registers: regRead and regWrite.

    It is up to the user to ensure that all addresses and values used in those two
    methods correspond to the above register classes.
    """
    finished = pyqtSignal()

    def __init__(self, ip=ETH_IP, port=ETH_PORT):
        super().__init__()
        self._ETH_IP = QHostAddress(ETH_IP)
        self._ETH_PORT = port
        self.version = 0

        # storage for retrieiving tcp data
        self.data = None

        # create the tcp socket
        self._tcpsocket = QTcpSocket(self)
        self._tcpConnected = self._tcp_connect()
        if self._tcpConnected:
            # connect the write command to reading if anything comes back
            self._tcpsocket.readyRead.connect(lambda: self._readData())

            # make sure to check this works
            # self._verify()

        # manage the SAQ-UDP thread reader
        self.thread = QThread()
        self.worker = UDPworker()
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.udp_done)

    def isConnected(self) -> bool:
        return self._tcpConnected

    def regRead(self, addr, cmd: str) -> int:
        """
        read a particular address from the Zynq
        """

        if isinstance(addr, list):
            args = [cmd, *addr]
        else:
            args = [cmd, addr]
            
        # form byte message
        if isinstance(args, str): args = args.split(' ')
        if len(cmd) < 4:
            hdr = args[0]+'\0'
        byte_arr = str.encode(hdr)
        for arg in args[1:]:
            if not isinstance(arg, int): arg = int(arg, 0)
            byte_arr += struct.pack('<I', arg)

        self._write(byte_arr)
        self._tcpsocket.waitForReadyRead(1000)

        # make sure there's new data to return
        if self.data is not None:
            data = self.data
            self.data = None
            return data
        else:
            print('WARNING: REG no data!')
            return -1

    def regWrite(self, addr, val, cmd='I2C') -> int:
        """
        write multiple possible bytes to the TCP socket.
        for infomation on how these packets are handled, see helper.c 
        in the zynq firmware.
        """
        print(f"writing {val:08x} to addr: {addr:08x}, cmd={cmd}")

        # form byte message
        args = [cmd, addr, val]
        if isinstance(args, str): args = args.split(' ')
        if len(cmd) < 4:
            hdr = args[0]+'\0'
        else:
            hdr = args[0]
        byte_arr = str.encode(hdr)
        for arg in args[1:]:
            if not isinstance(arg, int): arg = int(arg, 0)
            byte_arr += struct.pack('<I', arg)

        # returns number of bytes written
        print(f"sending bytes: {byte_arr}")
        cnt = self._write(byte_arr)
        self._tcpsocket.waitForReadyRead(1000)
        return cnt

    def _verify(self) -> bool:
        """
        initialization function to make sure that the interface can communicate
        with the scratch buffer.
        """
        pass
        # self.version = self.regRead(REG.SCRATCH)
        # print(f"Running version: 0x{self.version:08x}.. verifying..", end=" ")

        # # update and check
        # checksum = 0x0a0a_a0a0
        # self.regWrite(REG.SCRATCH, checksum)
        # verify  = self.regRead(REG.SCRATCH)
        # if checksum != verify:
        #     print("warning verification failed")
        #     print(f"0x{checksum:08x} != 0x{verify:08x}")
        # else:
        #     print("verification passed!")
        #     self.regWrite(REG.SCRATCH, self.version)

        # # set up SAQ register if version >= 8
        # if self.version <= LATEST_STABLE_VERSION:
        #     print("WARNING NOT A STABLE VERSION")

        # return checksum == verify

    def _readData(self) -> int:
        """
        PyQtSlot: Read data from the socket whenever something shows up.

        ARGS: opt: optional integer to fiure out which signal emitted call

        Returns the last 32 bit word from the socket, and handles tcp response
        """
        while self._tcpsocket.bytesAvailable():
            data = self._tcpsocket.read(4)
            val = struct.unpack('<I', data)[0]
            self.data = val
            print(f"{self.data:08x}")

        return self.data

    def _write(self, data):
        wrote = self._tcpsocket.write(data)
        self._tcpsocket.waitForBytesWritten(1000)
        return wrote

    def _tcp_connect(self):
        """
        connect to the remote socket and find the zybo board.
        """
        print("tcp connecting..", end="")
        connected = False
        # try to connect to the TCP Socket
        try:
            addr = QHostAddress(self._ETH_IP)
            self._tcpsocket.connectToHost(addr, self._ETH_PORT)
            print("TCP..", end=" ")
            if self._tcpsocket.waitForConnected(1000):
                print("connected..")
                connected = True
            else:
                print("WARNING unconnected..")

        except Exception as ex:
            print(ex)
            print("unconnected! TCP")

        return connected

    def udp_done(self):
        """
        signaled from the SAQ worker which manages the UDP socket
        """
        print("SAQ UDP thread worker is finished!")
        self.thread.quit()

    def start(self):
        """
        Used to start and stop the thread
        """
        self.thread.start()

    def finish(self):
        """
        slot function to emit finished signal
        """
        self.finished.emit()
        QUdpSocket().writeDatagram(EXIT_PACKET, QHostAddress(ETH_UDP_IP), ETH_UDP_PORT)


if __name__ == '__main__':
    pass
