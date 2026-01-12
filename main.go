package main

import (
	"bytes"
	"encoding/binary"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"runtime"
	"time"

	"github.com/tarm/serial"
)

const (
	connectionBaudRate          = 115200
	timeoutBetweenCommands      = 50 * time.Millisecond
	deviceNode                  = 0x0
	maxLenInBytes               = 21
	readTransmission       byte = 0b01
	writeTransmission      byte = 0b11
)

const (
	deviceTypeObject      = 0
	deviceSerialNoObject  = 1
	nominalVoltageObject  = 2
	nominalCurrentObject  = 3
	nominalPowerObject    = 4
	deviceArticleNoObject = 6
	manufacturerObject    = 8
	softwareVersionObject = 9
	setValueVoltageObject = 50
	setValueCurrentObject = 51
	powerSupplyControl    = 54
	statusActualValues    = 71
)

const (
	switchModeCmd    = 0x10
	switchModeRemote = 0x10
	switchModeManual = 0x00
	switchPowerCmd   = 0x01
	switchPowerOn    = 0x01
	switchPowerOff   = 0x00
)

func asString(raw []byte) string {
	return string(bytes.TrimRight(raw, "\x00"))
}

func asFloat(raw []byte) float64 {
	if len(raw) < 4 {
		return 0
	}
	return float64(math.Float32frombits(binary.BigEndian.Uint32(raw)))
}

func asWord(raw []byte) uint16 {
	if len(raw) < 2 {
		return 0
	}
	return binary.BigEndian.Uint16(raw)
}

func calcChecksum(payload []byte) []byte {
	var sum uint16
	for _, b := range payload {
		sum += uint16(b)
	}
	return []byte{byte(sum >> 8), byte(sum & 0xff)}
}

func startDelimiter(transmission byte, expectedLen byte) (byte, error) {
	if expectedLen == 0 || expectedLen > 16 {
		return 0, fmt.Errorf("expected data length must be 1..16, got %d", expectedLen)
	}
	result := (expectedLen - 1) | 0x10 | 0x20
	result |= transmission << 6
	return result, nil
}

type toPowerSupply struct {
	bytes      []byte
	checksum   []byte
	checksumOK bool
}

func newToPowerSupply(transmission byte, data []byte, expectedLen byte) (*toPowerSupply, error) {
	sd, err := startDelimiter(transmission, expectedLen)
	if err != nil {
		return nil, err
	}
	raw := append([]byte{sd}, data...)
	checksum := calcChecksum(raw)
	return &toPowerSupply{
		bytes:      raw,
		checksum:   checksum,
		checksumOK: true,
	}, nil
}

func (t *toPowerSupply) byteArray() []byte {
	return append(append([]byte{}, t.bytes...), t.checksum...)
}

type fromPowerSupply struct {
	bytes      []byte
	checksum   []byte
	checksumOK bool
}

func newFromPowerSupply(raw []byte) (*fromPowerSupply, error) {
	if len(raw) < 2 {
		return nil, errors.New("response too short")
	}
	body := raw[:len(raw)-2]
	checksum := raw[len(raw)-2:]
	checksumOK := bytes.Equal(checksum, calcChecksum(body))
	return &fromPowerSupply{
		bytes:      body,
		checksum:   checksum,
		checksumOK: checksumOK,
	}, nil
}

func (t *fromPowerSupply) data() []byte {
	if len(t.bytes) < 4 {
		return nil
	}
	return t.bytes[3:]
}

type DeviceInformation struct {
	DeviceType      string
	DeviceSerialNo  string
	NominalVoltage  float64
	NominalCurrent  float64
	NominalPower    float64
	Manufacturer    string
	DeviceArticleNo string
	SoftwareVersion string
}

func (d DeviceInformation) String() string {
	return fmt.Sprintf("%s %s [%s], SW: %s, Art-Nr: %s, [%.2f V, %.2f A, %.2f W]",
		d.Manufacturer, d.DeviceType, d.DeviceSerialNo, d.SoftwareVersion, d.DeviceArticleNo,
		d.NominalVoltage, d.NominalCurrent, d.NominalPower)
}

type DeviceStatusInformation struct {
	RemoteControlActive  bool
	OutputActive         bool
	ActualVoltagePercent float64
	ActualCurrentPercent float64
}

func newDeviceStatusInformation(raw []byte) DeviceStatusInformation {
	if len(raw) < 6 {
		return DeviceStatusInformation{}
	}
	return DeviceStatusInformation{
		RemoteControlActive:  raw[0]&0b1 == 1,
		OutputActive:         raw[1]&0b1 == 1,
		ActualVoltagePercent: float64(asWord(raw[2:4])) / 256,
		ActualCurrentPercent: float64(asWord(raw[4:6])) / 256,
	}
}

type PS2000B struct {
	port   io.ReadWriteCloser
	info   DeviceInformation
	status *DeviceStatusInformation
	open   bool
}

func OpenPS2000B(portName string) (*PS2000B, error) {
	cfg := &serial.Config{
		Name:        portName,
		Baud:        connectionBaudRate,
		Parity:      serial.ParityOdd,
		StopBits:    serial.Stop1,
		ReadTimeout: timeoutBetweenCommands * 2,
	}
	port, err := serial.OpenPort(cfg)
	if err != nil {
		return nil, err
	}

	ps := &PS2000B{
		port: port,
		open: true,
	}
	info, err := ps.readDeviceInformation()
	if err != nil {
		_ = port.Close()
		return nil, err
	}
	ps.info = info
	return ps, nil
}

func (p *PS2000B) Close() error {
	if !p.open {
		return nil
	}
	p.open = false
	return p.port.Close()
}

func (p *PS2000B) IsOpen() bool {
	return p.open
}

func (p *PS2000B) DeviceInformation() DeviceInformation {
	return p.info
}

func (p *PS2000B) readDeviceInformation() (DeviceInformation, error) {
	var info DeviceInformation

	data, err := p.readDeviceData(16, deviceTypeObject)
	if err != nil {
		return info, err
	}
	info.DeviceType = asString(data.data())

	data, err = p.readDeviceData(16, deviceSerialNoObject)
	if err != nil {
		return info, err
	}
	info.DeviceSerialNo = asString(data.data())

	data, err = p.readDeviceData(4, nominalVoltageObject)
	if err != nil {
		return info, err
	}
	info.NominalVoltage = asFloat(data.data())

	data, err = p.readDeviceData(4, nominalCurrentObject)
	if err != nil {
		return info, err
	}
	info.NominalCurrent = asFloat(data.data())

	data, err = p.readDeviceData(4, nominalPowerObject)
	if err != nil {
		return info, err
	}
	info.NominalPower = asFloat(data.data())

	data, err = p.readDeviceData(16, deviceArticleNoObject)
	if err != nil {
		return info, err
	}
	info.DeviceArticleNo = asString(data.data())

	data, err = p.readDeviceData(16, manufacturerObject)
	if err != nil {
		return info, err
	}
	info.Manufacturer = asString(data.data())

	data, err = p.readDeviceData(16, softwareVersionObject)
	if err != nil {
		return info, err
	}
	info.SoftwareVersion = asString(data.data())

	return info, nil
}

func (p *PS2000B) readDeviceData(expectedLen byte, objectID byte) (*fromPowerSupply, error) {
	telegram, err := newToPowerSupply(readTransmission, []byte{deviceNode, objectID}, expectedLen)
	if err != nil {
		return nil, err
	}
	return p.sendAndReceive(telegram.byteArray())
}

func (p *PS2000B) sendAndReceive(raw []byte) (*fromPowerSupply, error) {
	if _, err := p.port.Write(raw); err != nil {
		return nil, err
	}
	buf := make([]byte, maxLenInBytes)
	n, err := p.port.Read(buf)
	if err != nil {
		return nil, err
	}
	if n == 0 {
		return nil, errors.New("no response from device")
	}
	return newFromPowerSupply(buf[:n])
}

func (p *PS2000B) UpdateDeviceInformation() error {
	telegram, err := newToPowerSupply(readTransmission, []byte{deviceNode, statusActualValues}, 6)
	if err != nil {
		return err
	}
	deviceInformation, err := p.sendAndReceive(telegram.byteArray())
	if err != nil {
		return err
	}
	status := newDeviceStatusInformation(deviceInformation.data())
	p.status = &status
	return nil
}

func (p *PS2000B) DeviceStatusInformation() (*DeviceStatusInformation, error) {
	if p.status == nil {
		if err := p.UpdateDeviceInformation(); err != nil {
			return nil, err
		}
	}
	return p.status, nil
}

func (p *PS2000B) sendDeviceControl(p1 byte, p2 byte) error {
	telegram, err := newToPowerSupply(writeTransmission, []byte{deviceNode, powerSupplyControl, p1, p2}, 2)
	if err != nil {
		return err
	}
	if _, err := p.sendAndReceive(telegram.byteArray()); err != nil {
		return err
	}
	return p.UpdateDeviceInformation()
}

func (p *PS2000B) sendDeviceData(obj byte, data uint16) error {
	telegram, err := newToPowerSupply(writeTransmission, []byte{deviceNode, obj, byte(data >> 8), byte(data & 0xff)}, 4)
	if err != nil {
		return err
	}
	if _, err := p.sendAndReceive(telegram.byteArray()); err != nil {
		return err
	}
	return p.UpdateDeviceInformation()
}

func (p *PS2000B) EnableRemoteControl() error {
	return p.sendDeviceControl(switchModeCmd, switchModeRemote)
}

func (p *PS2000B) DisableRemoteControl() error {
	return p.sendDeviceControl(switchModeCmd, switchModeManual)
}

func (p *PS2000B) EnableOutput() error {
	return p.sendDeviceControl(switchPowerCmd, switchPowerOn)
}

func (p *PS2000B) DisableOutput() error {
	return p.sendDeviceControl(switchPowerCmd, switchPowerOff)
}

func (p *PS2000B) Output() (bool, error) {
	status, err := p.DeviceStatusInformation()
	if err != nil {
		return false, err
	}
	return status.OutputActive, nil
}

func (p *PS2000B) Voltage() (float64, error) {
	if err := p.UpdateDeviceInformation(); err != nil {
		return 0, err
	}
	if p.status == nil {
		return 0, errors.New("device status not available")
	}
	voltage := p.info.NominalVoltage * p.status.ActualVoltagePercent
	return voltage / 100, nil
}

func (p *PS2000B) VoltageSetpoint() (float64, error) {
	res, err := p.readDeviceData(2, setValueVoltageObject)
	if err != nil {
		return 0, err
	}
	data := res.data()
	if len(data) < 2 {
		return 0, errors.New("invalid voltage setpoint response")
	}
	raw := (uint16(data[0]) << 8) + uint16(data[1])
	return p.info.NominalVoltage * float64(raw) / 25600.0, nil
}

func (p *PS2000B) SetVoltage(value float64) error {
	if err := p.UpdateDeviceInformation(); err != nil {
		return err
	}
	if err := p.EnableRemoteControl(); err != nil {
		return err
	}
	volt := uint16(math.Round((value * 25600.0) / p.info.NominalVoltage))
	return p.sendDeviceData(setValueVoltageObject, volt)
}

func (p *PS2000B) Current() (float64, error) {
	if err := p.UpdateDeviceInformation(); err != nil {
		return 0, err
	}
	if p.status == nil {
		return 0, errors.New("device status not available")
	}
	current := p.info.NominalCurrent * p.status.ActualCurrentPercent
	return current / 100, nil
}

func (p *PS2000B) CurrentSetpoint() (float64, error) {
	res, err := p.readDeviceData(2, setValueCurrentObject)
	if err != nil {
		return 0, err
	}
	data := res.data()
	if len(data) < 2 {
		return 0, errors.New("invalid current setpoint response")
	}
	raw := (uint16(data[0]) << 8) + uint16(data[1])
	return p.info.NominalCurrent * float64(raw) / 25600.0, nil
}

func (p *PS2000B) SetCurrent(value float64) error {
	if err := p.UpdateDeviceInformation(); err != nil {
		return err
	}
	if err := p.EnableRemoteControl(); err != nil {
		return err
	}
	curr := uint16(math.Round((value * 25600.0) / p.info.NominalCurrent))
	return p.sendDeviceData(setValueCurrentObject, curr)
}

type Mypower struct {
	device *PS2000B
}

func (m *Mypower) Connect(portName string) error {
	device, err := OpenPS2000B(portName)
	if err != nil {
		return err
	}
	m.device = device
	return nil
}

func (m *Mypower) Close() error {
	if m.device == nil {
		return nil
	}
	return m.device.Close()
}

func (m *Mypower) OpenPower() error {
	if m.device == nil {
		return errors.New("device not connected")
	}
	if err := m.device.EnableRemoteControl(); err != nil {
		return err
	}
	return m.device.EnableOutput()
}

func (m *Mypower) ClosePower() error {
	if m.device == nil {
		return errors.New("device not connected")
	}
	return m.device.DisableOutput()
}

func (m *Mypower) ControlPower(voltage float64, current float64) error {
	if m.device == nil {
		return errors.New("device not connected")
	}
	if err := m.device.SetVoltage(voltage); err != nil {
		return err
	}
	return m.device.SetCurrent(current)
}

func (m *Mypower) GetV() (float64, error) {
	if m.device == nil {
		return 0, errors.New("device not connected")
	}
	return m.device.Voltage()
}

func main() {
	portFlag := flag.String("port", "", "serial port (COM3 or /dev/ttyACM0)")
	voltage := flag.Float64("voltage", 13.5, "voltage setpoint")
	current := flag.Float64("current", 3, "current setpoint")
	toggle := flag.Bool("toggle", false, "toggle output every 2s")
	flag.Parse()

	portName := *portFlag
	if portName == "" {
		if runtime.GOOS == "windows" {
			fmt.Print("Enter COM number: ")
			var n int
			if _, err := fmt.Scan(&n); err == nil && n > 0 {
				portName = fmt.Sprintf("COM%d", n)
			} else {
				portName = "COM1"
			}
		} else {
			portName = "/dev/ttyACM0"
		}
	}

	power := &Mypower{}
	fmt.Printf("Connecting to %s...\n", portName)
	if err := power.Connect(portName); err != nil {
		log.Fatalf("connect failed: %v", err)
	}
	defer func() {
		if err := power.Close(); err != nil {
			log.Printf("close failed: %v", err)
		}
	}()

	fmt.Printf("Connected: %v\n", power.device.IsOpen())
	fmt.Printf("Device info: %s\n", power.device.DeviceInformation())

	if err := power.OpenPower(); err != nil {
		log.Fatalf("open power failed: %v", err)
	}
	if err := power.ControlPower(*voltage, *current); err != nil {
		log.Fatalf("set power failed: %v", err)
	}

	if *toggle {
		for {
			time.Sleep(2 * time.Second)
			_ = power.ClosePower()
			time.Sleep(2 * time.Second)
			_ = power.OpenPower()
		}
	}
}
