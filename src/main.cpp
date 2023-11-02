#include <Arduino.h>

#include <SdFat.h>
#include <ADC.h>
#include <sdios.h>
#include <vector>
#include <algorithm>
#include <CircularBuffer.h>

//#define SD_INFO

// set up variables using the SD utility library functions:
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

// Try to select the best SD card configuration.
#define SD_CONFIG SdioConfig(FIFO_SDIO)

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE

//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))
//------------------------------------------------------------------------------
// Check for extra characters in field or find minus sign.
char *skipSpace(char *str)
{
	while (isspace(*str))
		str++;
	return str;
}

cid_t m_cid;
csd_t m_csd;
uint32_t m_eraseSize;
uint32_t m_ocr;

// CSV config
String label_csv_name = "datasets.csv";
String first_raw_data = "x:sensor,y";

const int class_sumple_n = 20;
std::vector<String> folder_struct = {"/110001", "/210001"};
std::vector<int> file_init_number(folder_struct.size(), 0);
std::vector<int> class_sample(folder_struct.size(), class_sumple_n);

static ArduinoOutStream cout(Serial);

// ADC
ADC *adc = new ADC(); // adc object
std::vector<int> adc_pins = {A9, A7, A3};
const uint8_t sens_n = adc_pins.size();

// sample valiable
const uint_least16_t average_n = 100;
const uint_least16_t threshold = 240;
const uint_least16_t sample_n = 9600;
const uint_least16_t preRize_sample_n = 5000;
const uint_least16_t postRize_sample_n = sample_n - preRize_sample_n;
std::vector<CircularBuffer<uint_least16_t, sample_n>> buffer(sens_n);

// initialize average
std::vector<uint_least16_t> init_average(3, 0);

// measurement ON or OFF switch
const uint8_t sw = 8;
elapsedMicros timer = 0;
elapsedMicros measure_time = 0;

uint_least16_t save_cout_rate = 10000;

//------------------------------------------------------------------------------
#ifdef SD_INFO
bool cidDmp()
{
	cout << F("\nManufacturer ID: ");
	cout << uppercase << showbase << hex << int(m_cid.mid) << dec << endl;
	cout << F("OEM ID: ") << m_cid.oid[0] << m_cid.oid[1] << endl;
	cout << F("Product: ");
	for (uint8_t i = 0; i < 5; i++)
	{
		cout << m_cid.pnm[i];
	}
	cout << F("\nVersion: ");
	cout << int(m_cid.prv_n) << '.' << int(m_cid.prv_m) << endl;
	cout << F("Serial number: ") << hex << m_cid.psn << dec << endl;
	cout << F("Manufacturing date: ");
	cout << int(m_cid.mdt_month) << '/';
	cout << (2000 + m_cid.mdt_year_low + 10 * m_cid.mdt_year_high) << endl;
	cout << endl;
	return true;
}
//------------------------------------------------------------------------------
void clearSerialInput()
{
	uint32_t m = micros();
	do
	{
		if (Serial.read() >= 0)
		{
			m = micros();
		}
	} while (micros() - m < 10000);
}
//------------------------------------------------------------------------------
bool csdDmp()
{
	bool eraseSingleBlock;
	if (m_csd.v1.csd_ver == 0)
	{
		eraseSingleBlock = m_csd.v1.erase_blk_en;
		m_eraseSize = (m_csd.v1.sector_size_high << 1) | m_csd.v1.sector_size_low;
	}
	else if (m_csd.v2.csd_ver == 1)
	{
		eraseSingleBlock = m_csd.v2.erase_blk_en;
		m_eraseSize = (m_csd.v2.sector_size_high << 1) | m_csd.v2.sector_size_low;
	}
	else
	{
		cout << F("m_csd version error\n");
		return false;
	}
	m_eraseSize++;
	cout << F("cardSize: ") << 0.000512 * sdCardCapacity(&m_csd);
	cout << F(" MB (MB = 1,000,000 bytes)\n");

	cout << F("flashEraseSize: ") << int(m_eraseSize) << F(" blocks\n");
	cout << F("eraseSingleBlock: ");
	if (eraseSingleBlock)
	{
		cout << F("true\n");
	}
	else
	{
		cout << F("false\n");
	}
	return true;
}
//------------------------------------------------------------------------------
void errorPrint()
{
	if (sd.sdErrorCode())
	{
		cout << F("SD errorCode: ") << hex << showbase;
		printSdErrorSymbol(&Serial, sd.sdErrorCode());
		cout << F(" = ") << int(sd.sdErrorCode()) << endl;
		cout << F("SD errorData = ") << int(sd.sdErrorData()) << endl;
	}
}
//------------------------------------------------------------------------------
bool mbrDmp()
{
	MbrSector_t mbr;
	bool valid = true;
	if (!sd.card()->readSector(0, (uint8_t *)&mbr))
	{
		cout << F("\nread MBR failed.\n");
		errorPrint();
		return false;
	}
	cout << F("\nSD Partition Table\n");
	cout << F("part,boot,bgnCHS[3],type,endCHS[3],start,length\n");
	for (uint8_t ip = 1; ip < 5; ip++)
	{
		MbrPart_t *pt = &mbr.part[ip - 1];
		if ((pt->boot != 0 && pt->boot != 0X80) ||
			getLe32(pt->relativeSectors) > sdCardCapacity(&m_csd))
		{
			valid = false;
		}
		cout << int(ip) << ',' << uppercase << showbase << hex;
		cout << int(pt->boot) << ',';
		for (int i = 0; i < 3; i++)
		{
			cout << int(pt->beginCHS[i]) << ',';
		}
		cout << int(pt->type) << ',';
		for (int i = 0; i < 3; i++)
		{
			cout << int(pt->endCHS[i]) << ',';
		}
		cout << dec << getLe32(pt->relativeSectors) << ',';
		cout << getLe32(pt->totalSectors) << endl;
	}
	if (!valid)
	{
		cout << F("\nMBR not valid, assuming Super Floppy format.\n");
	}
	return true;
}
//------------------------------------------------------------------------------
void dmpVol()
{
	cout << F("\nScanning FAT, please wait.\n");
	uint32_t freeClusterCount = sd.freeClusterCount();
	if (sd.fatType() <= 32)
	{
		cout << F("\nVolume is FAT") << int(sd.fatType()) << endl;
	}
	else
	{
		cout << F("\nVolume is exFAT\n");
	}
	cout << F("sectorsPerCluster: ") << sd.sectorsPerCluster() << endl;
	cout << F("clusterCount:      ") << sd.clusterCount() << endl;
	cout << F("freeClusterCount:  ") << freeClusterCount << endl;
	cout << F("fatStartSector:    ") << sd.fatStartSector() << endl;
	cout << F("dataStartSector:   ") << sd.dataStartSector() << endl;
	if (sd.dataStartSector() % m_eraseSize)
	{
		cout << F("Data area is not aligned on flash erase boundary!\n");
		cout << F("Download and use formatter from www.sdcard.org!\n");
	}
}
//------------------------------------------------------------------------------
void printCardType()
{

	cout << F("\nCard type: ");

	switch (sd.card()->type())
	{
	case SD_CARD_TYPE_SD1:
		cout << F("SD1\n");
		break;

	case SD_CARD_TYPE_SD2:
		cout << F("SD2\n");
		break;

	case SD_CARD_TYPE_SDHC:
		if (sdCardCapacity(&m_csd) < 70000000)
		{
			cout << F("SDHC\n");
		}
		else
		{
			cout << F("SDXC\n");
		}
		break;

	default:
		cout << F("Unknown\n");
	}
}
#endif
//------------------------------------------------------------------------------

String sliceData(const std::vector<CircularBuffer<uint_least16_t, sample_n>> *data, const uint_least16_t iter)
{
	String sliceData_ = "";
	for (int i = 0; i < sens_n; i++)
	{
		sliceData_ += String(data->at(i)[iter]);
		if (i < (sens_n - 1))
			sliceData_ += ",";
	}
	return sliceData_;
}

bool openFile(FsFile* file, const char *path)
{
	if (file->open(path, FILE_WRITE)){
		cout << "opened: " << path << endl;
		return true;
	}
	else{
		cout << "error:"
			 << "can't open file" << endl;
		return false;
	}
}

bool closeFile(FsFile* file, bool monitor = false)
{
	if (file->close())
	{
		if(monitor == true)
		{
			cout << "closed file" << endl;
		}
		return true;
	}
	else{
		cout << "error:"
			 << "can't close file" << endl;
		return false;
	}
}

bool saveData(FsFile *file, const String data, bool monitor = false)
{
	// if the file is available, write to it:
	if (file)
	{
		file->println(data);

		//  print to the` serial port too:
		if (monitor == true)
		{
			cout << "Wrote: " << data << endl;
		}
		return true;
	}
	// if the file isn't open, pop up an error:
	else
	{
		cout << "error opening " << endl;
		return false;
	}
}

bool readbyte(FsFile *file, String &data, uint64_t bytePos = 0, bool monitor = false)
{
	char cdata[50];
	file->seek(bytePos);

	// if the file is available, write to it:
	if (file)
	{
		if (file->available() > 0)
		{
			int n = file->fgets(cdata, sizeof(cdata));
			if (n <= 0)
			{
				error("fgets failed");
			}
			if (cdata[n - 1] != '\n' && n == (sizeof(cdata) - 1))
			{
				error("line too long");
			}
		}

		data = String(cdata);

		//  print to the` serial port too:
		if (monitor == true)
			cout << "Read: " << data << endl;

		return true;
	}
	// if the file isn't open, pop up an error:
	else
	{
		cout << "error opening " << endl;
		return false;
	}
}

bool readData(FsFile *file, String &data, uint16_t rowPos, uint16_t rowbyte, bool monitor = false)
{
	char cdata[50];
	file->seek((rowPos - 1) * rowbyte + 1);

	// if the file is available, write to it:
	if (file)
	{
		if (file->available() > 0)
		{
			int n = file->fgets(cdata, sizeof(cdata));
			if (n <= 0)
			{
				error("fgets failed");
			}
			if (cdata[n - 1] != '\n' && n == (sizeof(cdata) - 1))
			{
				error("line too long");
			}
		}

		data = String(cdata);

		//  print to the` serial port too:
		if (monitor == true)
			cout << "Read: " << data << endl;

		return true;
	}
	// if the file isn't open, pop up an error:
	else
	{
		cout << "error opening " << endl;
		return false;
	}
}

bool folder_struct_check()
{
	size_t successful = 0;
	for (int index = 0; index < (int)folder_struct.size(); index++)
	{
		if (!sd.exists(folder_struct.at(index)))
		{
			successful += sd.mkdir(folder_struct.at(index));
		}
	}
	if (successful == folder_struct.size())
	{
		return true;
	}
	else
	{
		return false;
	}
}

void label_csv_file_check()
{
	if (!sd.exists(label_csv_name))
	{
		openFile(&file, label_csv_name.c_str());
		saveData(&file, first_raw_data, true);
		closeFile(&file);
	}
}

void file_init_number_check(bool monitor = false)
{
	for (int index = 0; index < (int)folder_struct.size(); index++)
	{
		int file_number = 0;
		String file_name = "";
		do
		{
			file_number++;
			file_name = folder_struct.at(index) + "/" + String(file_number) + ".csv";
		} while (sd.exists(file_name));

		file_init_number.at(index) = file_number;
		if (monitor)
		{
			cout << folder_struct.at(index) << ": start " << file_init_number.at(index) << ".csv" << endl;
		}
	}
}

void endProcess()
{
	while (true)
	{
		yield();
	}
}

void setup()
{
	// UNCOMMENT THESE TWO LINES FOR TEENSY AUDIO BOARD:
	// SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
	// SPI.setSCK(14);  // Audio shield has SCK on pin 14

	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while (!Serial)
	{
		SysCall::yield(); // wait for serial port to connect.
	}

	pinMode(sw, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWriteFast(LED_BUILTIN, LOW);

	for (int i = 0; i < sens_n; i++)
	{
		pinMode(adc_pins[i], INPUT);
	}

	///// ADC0 ////
	adc->adc0->setAveraging(1);											  // set number of averages
	adc->adc0->setResolution(12);										  // set bits of resolution
	adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
	adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);	  // change the sampling speed

////// ADC1 /////
#ifdef ADC_DUAL_ADCS
	adc->adc1->setAveraging(1);											  // set number of averages
	adc->adc1->setResolution(12);										  // set bits of resolution
	adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
	adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);	  // change the sampling speed
#endif

#ifdef SD_INFO
	// Read any existing Serial data.
	clearSerialInput();

	// F stores strings in flash to save RAM
	cout << F("\ntype any character to start\n");
	while (!Serial.available())
	{
		SysCall::yield();
	}
	uint32_t t = millis();
	if (!sd.cardBegin(SD_CONFIG))
	{
		cout << F(
			"\nSD initialization failed.\n"
			"Do not reformat the card!\n"
			"Is the card correctly inserted?\n"
			"Is there a wiring/soldering problem?\n");
		if (isSpi(SD_CONFIG))
		{
			cout << F(
				"Is SD_CS_PIN set to the correct value?\n"
				"Does another SPI device need to be disabled?\n");
		}
		errorPrint();
		return;
	}
	t = millis() - t;
	cout << F("init time: ") << t << " ms" << endl;

	if (!sd.card()->readCID(&m_cid) ||
		!sd.card()->readCSD(&m_csd) ||
		!sd.card()->readOCR(&m_ocr))
	{
		cout << F("readInfo failed\n");
		errorPrint();
		return;
	}

	printCardType();
	cidDmp();
	csdDmp();
	cout << F("\nOCR: ") << uppercase << showbase;
	cout << hex << m_ocr << dec << endl;
	if (!mbrDmp())
	{
		return;
	}
	if (!sd.volumeBegin())
	{
		cout << F("\nvolumeBegin failed. Is the card formatted?\n");
		errorPrint();
		return;
	}
	dmpVol();
	while (true)
	{
		yield();
	}
#else
	Serial.print("Initializing SD card...");
	// see if the card is present and can be initialized:
	if (!sd.begin(SD_CONFIG))
	{
		sd.initErrorHalt(&Serial);
		// don't do anything more:
		return;
	}
	Serial.println("card initialized.");

	folder_struct_check();
	label_csv_file_check();
	file_init_number_check(true);
	delay(1000);
#endif
}

void loop()
{
	for (int folder_i = 0; folder_i < (int)folder_struct.size(); folder_i++)
	{
		cout << "measurement start:" << folder_struct.at(folder_i) << endl;
		for (int file_i = 0; file_i < class_sample.at(folder_i); file_i++)
		{
			int count = 0;
			bool start = false;
			bool start_sw = false;
			bool init_ave = false;
			String file_name = folder_struct.at(folder_i) + "/" + String(file_i + file_init_number.at(folder_i)) + ".csv";

			// label CSV
			openFile(&file, label_csv_name.c_str());
			saveData(&file, "." + file_name + "," + String(folder_i), true);
			closeFile(&file);

			

			do
			{
				if (timer > 9)
				{
					timer = 0;

					// read three sensors and append to the string:
					for (int i = 0; i < sens_n; i++)
					{
						uint_least16_t sensor_val = adc->analogRead(adc_pins[i]);
						buffer.at(i).push(sensor_val);
						if (start == false && buffer.at(0).size() == sample_n)
						{
							digitalWrite(LED_BUILTIN, HIGH);
							start_sw = start_sw==true ? true : !digitalRead(sw);
							
							if (start_sw == true && init_ave == false)
							{
								init_ave = true;
								for(int i = 0; i < sens_n; i++)
								{
									int sensor_sum = 0;
									for(int j = 0; j < average_n; j++)
									{
										sensor_sum += adc->analogRead(adc_pins[i]);
									}
									init_average.at(i) = sensor_sum / average_n;
									cout << init_average.at(i) << " ";
								}
								cout << endl;
							}

							if(start_sw == true && abs(init_average.at(i)-sensor_val) > threshold)
							{
								start = true;
								init_ave = false;
								digitalWrite(LED_BUILTIN, LOW);
								cout << "RISE Detection!!" << sensor_val - init_average.at(i) << endl;
								measure_time = 0;
							}
						}
					}

					if (start == true)
						count++;
				}
			} while (count != postRize_sample_n);

			cout << "measurement time: " << measure_time << endl;
			// open the file.
			measure_time = 0;
			openFile(&file, file_name.c_str());
			for (uint_least16_t iter = 0; iter < sample_n; iter++)
			{
				saveData(&file, sliceData(&buffer, iter), iter % save_cout_rate == 0 ? true : false);
			}
			closeFile(&file);
			cout << " elapsed time: " << measure_time << "us" << endl;
			cout << " Write rate: " << measure_time / sample_n << "us/S" << endl << endl;
		}
	}
	cout << "measurement end" << endl;
	endProcess();
}
