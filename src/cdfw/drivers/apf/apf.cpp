/*
 * Copyright 2022 Murray Aickin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

#include "hardware.h"
#include "apf.h"
#include "riscprintf.h"
#include "timer.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SSPI_ACK

// bool file_error_enabled;
// int file_error_code;

bool dataslot_ready(){
  int apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
  if ((apf_codes & APF_ERROR) > 0) mainprintf("\033[38;1;5mCheck codes %0.4X\033[0m\r\n", (apf_codes & APF_ERROR));
  // if (!(apf_codes & APF_DONE)) mainprintf("\033[38;1;5mwe are waiting\033[0m\r\n");
  // file_error_enabled = 1;
  // file_error_code = error_info_dataslot_Faild;
  return(apf_codes & APF_DONE);
}

uint8_t dataslot_status(){

  int apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
  return(apf_codes & 0x7);
}

// This is to search though the dataslot ram looking for the location of the data slot in it
uint32_t dataslot_search_id(uint16_t value)
{
    int i = 0;
    // while loop from 1 to 5
    do  {
      if (DATASLOT_RAM_ACCESS(i<<2) == value){
        return (i);
      };
      i=i+2;
    } while (i <= 0x100);
    return (0);
}

// This is to search though the dataslot ram on what the sizes of the dataslow wanted
uint32_t dataslot_size(uint16_t value)
{
  int i;
  uint32_t size;
  mainprintf("number value: %d \r\n", value);
  i = dataslot_search_id(value);
  // riscprintf("number slot: %d \r\n", i);
  size = DATASLOT_RAM_ACCESS((i+1)<<2);
  mainprintf("number size: %d \r\n", size);
  return (size);
}

// This checks the reg in the system on if there was an apf dataslot update. Once this reg is read it is cleared by hardware
bool dataslot_updated()
{
    return(DATASLOT_UPDATE_REG(0));
}

// This will send the read command to the APF
// The WRITE_* are the regs in the core for the location of the dataslow wanted to be read
// This is used for block data with no pointer changes

uint32_t dataslot_read(uint16_t dataslot, uint32_t address, uint32_t offset, uint32_t length)
{

  WRITE_TARGET_DATASLOT_ID(0) = dataslot;
  WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = address;
  if (length > 0)WRITE_TARGET_DATASLOT_LENGTH(0) = length;
  else return(0);
  WRITE_TARGET_DATASLOT_OFFSET(0) = offset;
  WRITE_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_READ_REG;
  mainprintf("APF read: %d, %0.4x, %0.4x, %d\r\n", dataslot, WRITE_TARGET_DATASLOT_BRIDGE_ADD(0), offset, length);
  int apf_codes;
  int i = 0;
  do
  {
    apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
    {
      mainprintf("\033[36;5;4mAPF FAILD dataslot_read check? dataslot %d, address %0.4X, offset%0.4X, length%0.4X error %0.4X \033[0m\r\n", dataslot, address, offset, length, apf_codes);
      file_error_enabled = 1;
      file_error_code = error_info_dataslot_Load_Faild;
      return (apf_codes & APF_ERROR);
    }
    i++;
  } while (!(apf_codes & APF_ACK) | (i <= 1000));

	do
	{
		apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
		{
			mainprintf("\033[36;5;4mAPF FAILD dataslot_read done? dataslot %d, address %0.4X, offset %0.4X, length %0.4X error %0.4X\033[0m\r\n", dataslot, address, offset, length, apf_codes);
			
      file_error_enabled = 1;
      file_error_code = error_info_dataslot_Load_Faild;
      return (apf_codes & APF_ERROR);
		}
	} while (!(apf_codes & APF_DONE));
  mainprintf("\033[38;1;5mdone\033[0m\r\n");
  return(0);
}

// This will send the write command to the APF and send back a error if true
// This is used for block data with no pointer changes

uint32_t dataslot_write(uint16_t dataslot, uint32_t address, uint32_t offset, uint32_t length)
{
  WRITE_TARGET_DATASLOT_ID(0) = dataslot;
  WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = address;
  if (length > 0)WRITE_TARGET_DATASLOT_LENGTH(0) = length;
  else return(0);
  WRITE_TARGET_DATASLOT_OFFSET(0) = offset;
  WRITE_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_WRITE_REG;
  int apf_codes;
	do
	{
		apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
		{
			// riscprintf("\033[36;5;4mAPF FAILD dataslot_write? dataslot %d, address %0.4X, offset%0.4X, length%0.4X\033[0m\r\n", dataslot, address, offset, length);
			file_error_enabled = 1;
      file_error_code = error_info_dataslot_save_Faild;
      return (apf_codes & APF_ERROR);
		}
	} while (!(apf_codes & APF_DONE));
  return(0);
}

// THis will set the pointers read to go on the APF regs
uint32_t dataslot_read_lba_set_fast(uint16_t dataslot, uint32_t address, uint32_t offset, uint32_t length)
{
  int i = 0;
  int apf_codes;
  do
	{
    apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
    i++;
  } while (!(apf_codes & APF_DONE) | (i <=10000));

  READ_TARGET_DATASLOT_ID(0) = dataslot;
  READ_TARGET_DATASLOT_BRIDGE_ADD(0) = address;
  READ_TARGET_DATASLOT_OFFSET(0) = offset << 9;
  if (length > 0)READ_TARGET_DATASLOT_LENGTH(0) = length;
  else return(0);
  READ_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_READ_REG;

  mainprintf("\033[38;1;6mREAD_TARGET_DATASLOT_SETUP fast %0.4x, %0.4x, %0.4x, %d \033[0m\r\n", offset, address, length, dataslot);

  do
	{
    apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
  } while (!(apf_codes & APF_ACK) );

  int tmp = READ_TARGET_DATASLOT_OFFSET(0);
  READ_TARGET_DATASLOT_OFFSET(0) = tmp + length;
  return(0);
}



// THis will set the pointers read to go on the APF regs
uint32_t dataslot_read_lba_set(uint16_t dataslot, uint32_t address, uint32_t offset)
{
  READ_TARGET_DATASLOT_ID(0) = dataslot;
  READ_TARGET_DATASLOT_BRIDGE_ADD(0) = address;
  READ_TARGET_DATASLOT_OFFSET(0) = offset;

  mainprintf("\033[38;1;6mREAD_TARGET_DATASLOT_SETUP Offset %0.4x, %0.4x, %d \033[0m\r\n", offset, address, dataslot);
  return(0);
}

// This will read the data from where the file pointer is (This is set by dataslot_read_lba_set)
uint32_t dataslot_read_lba(uint32_t length)
{
  if (length > 0)READ_TARGET_DATASLOT_LENGTH(0) = length;
  else return(0);
  READ_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_READ_REG;
  mainprintf("READ_TARGET_DATASLOT_LENGTH Sent %.4x\r\n", WRITE_TARGET_DATASLOT_LENGTH(0));
  int i = 0;
  int apf_codes;
  do
	{
    apf_codes = READ_TARGET_DATASLOT_CONTROL(0);

    i++;
  } while (!(apf_codes & APF_ACK) | (i <=100));
	do
	{
		apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
		{
			mainprintf("\033[36;5;4mAPF FAILD read_lba? address %0.4X, Offset %0.4X, length %0.4X\033[0m\r\n", READ_TARGET_DATASLOT_BRIDGE_ADD(0), READ_TARGET_DATASLOT_OFFSET(0), length);
			return (apf_codes & APF_ERROR);
		}
    // minimig_input_update();
	} while (!(apf_codes & APF_DONE));
  int tmp = WRITE_TARGET_DATASLOT_OFFSET(0);
  WRITE_TARGET_DATASLOT_OFFSET(0) = tmp + length;
  return(0);
}

// This will read the data from where the file pointer is (This is set by dataslot_read_lba_set) and does the next batch of data
uint32_t dataslot_read_lba_fast( uint32_t length, bool wait_update)
{
  int apf_codes;
  int i = 0;
  if (length > 0)READ_TARGET_DATASLOT_LENGTH(0) = length;
  else return(0);
  READ_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_READ_REG;

  // mainprintf("READ_TARGET_DATASLOT_LENGTH fast Sent %.4x\r\n", length);
  if (wait_update) {
    do
  	{
      apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
      i++;
    } while (!(apf_codes & APF_ACK) | (i <=10000));
    int tmp = WRITE_TARGET_DATASLOT_OFFSET(0);
    WRITE_TARGET_DATASLOT_OFFSET(0) = tmp + length;
    // uint32_t address = READ_TARGET_DATASLOT_BRIDGE_ADD(0);
    // WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = address + length;
  }
  return(0);
}

// THis will set the pointers write to go on the APF regs
uint32_t dataslot_write_lba_set(uint16_t dataslot, uint32_t address, uint32_t offset)
{
  WRITE_TARGET_DATASLOT_ID(0) = dataslot;
  WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = address;
  WRITE_TARGET_DATASLOT_OFFSET(0) = offset << 9;
  WRITE_TARGET_DATASLOT_LENGTH(0) = 1;
  WRITE_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_WRITE_REG;

    // mainprintf("dataslot_write_lba_set address %.4x offset %.4x\r\n", address, offset);
  int apf_codes;
	do
	{
		apf_codes = READ_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
		{
			// riscprintf("\033[36;5;4mAPF FAILD dataslot_write_lba_set? dataslot %d, address %0.4X, offset%0.4X\033[0m\r\n", dataslot, address, offset);
			return (apf_codes & APF_ERROR);
		}
	} while (!(apf_codes & APF_DONE));
  return(0);
}

// This will write the data from where the file pointer is (This is set by dataslot_read_lba_set)
uint32_t dataslot_write_lba(uint32_t length, bool update_address)
{
  if (length > 0)WRITE_TARGET_DATASLOT_LENGTH(0) = length;
  else return(0);
  WRITE_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_WRITE_REG;
  int apf_codes;
	do
	{
		apf_codes = WRITE_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
		{
			// riscprintf("\033[36;5;4mAPF FAILD dataslot_write_lba? length%0.4X\033[0m\r\n", length);
      
			return (apf_codes & APF_ERROR);
		}
    // minimig_input_update();
	} while (!(apf_codes & APF_DONE));
  int tmp = WRITE_TARGET_DATASLOT_OFFSET(0);
  WRITE_TARGET_DATASLOT_OFFSET(0) = tmp + length;
  uint32_t address = READ_TARGET_DATASLOT_BRIDGE_ADD(0);
  if (update_address) WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = address + length;
  return(0);
}

// we need a check commands for the APF

dataslot_type aft;
char data [255];

void dataslot_pathname_file_decode(dataslot_type *aft){
  int L = strlen(data);
  int path_count = 0;
  for(int i = L; i--; i > 0){
    if(data[i] == '/'){
      path_count = i;
      break;
    }
  }
  
  mainprintf("dataslot_pathname_file_decode %d %s %d\n\r",L, data, path_count);
  
  
  uint32_t i = 0;

  do {
    aft->aft_filename[i] = 0;
    aft->aft_filepath[i] = 0;
    asm volatile("");
    i++;
  }while(i <= sizeof(aft->aft_filepath));
  
  mainprintf("File Name %s\r\n", aft->aft_filename);
  strcpy(aft->aft_filename, &data[path_count+1]);
  strncpy(aft->aft_filepath, data, path_count+1); 
  mainprintf("File Path %s\r\n", aft->aft_filepath);
  mainprintf("Data Slot Decode done\r\n");
}


int GetFileNameDataSlot (dataslot_type *aft){

  uint32_t temp = MAIN_SCRACH_ADDRESS_OFFSET;
	uint32_t * foo1 = &temp;
  mainprintf("dataslot Set Filename \r\n");
  
  WRITE_TARGET_DATASLOT_ID(0) = aft->aft_dataslot;
  WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = APF_SCRACH_ADDRESS_OFFSET;
  WRITE_TARGET_DATASLOT_OFFSET(0) = APF_SCRACH_ADDRESS_OFFSET;
  WRITE_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_GET_FILENAME_REG;
  uint32_t apf_codes;
  do
	{
		apf_codes = WRITE_TARGET_DATASLOT_CONTROL(0);
    if ((apf_codes & APF_ERROR) > 0)
		{
      file_error_enabled = 1;
      file_error_code = error_info_dataslot_File_Not_Found;
			mainprintf("\033[36;5;4mAPF FAILD File Name Dataslot? length%0.4X\033[0m\r\n", apf_codes & APF_ERROR);
      // error_dataslot = aft_dataslot;
			return (apf_codes & APF_ERROR);
		}
    // minimig_input_update();
	} while (!(apf_codes & APF_DONE));

  aft->aft_op_filesize  =  dataslot_size(aft->aft_dataslot);
  aft->aft_op_flags     =  SCRACH_RAMBUFFER(260);

  uint32_t i = 0;
  
  do {
    data[i] = (char)SCRACH_RAMBUFFER_BYTE(i);
    i++;
  }while (i < 254);
  
  mainprintf("File Dataslot Data %s\n\r",data);

  dataslot_pathname_file_decode(aft);

  mainprintf("Directory %s \r\n File name %s \r\n more information %0.8x %0.8x\r\n",aft->aft_filepath, aft->aft_filename, aft->aft_op_flags, aft->aft_op_filesize);
  // mainprintf("\r\n");

  return 0;
}

int OpenFileNameDataSlot (dataslot_type *aft, const char *s){
    
    uint32_t temp = MAIN_SCRACH_ADDRESS_OFFSET;
	  uint32_t * foo1 = &temp;
    int count = strlen(s);
    mainprintf("dataslot Open  %d count %d\r\n", aft->aft_dataslot, count);
    // memcpy(foo1, s, strlen(s));
    uint8_t i = 0;

    do {
      SCRACH_RAMBUFFER_BYTE(i) = 0;
      i++;
    }while (i < 255);

    i = 0;
    do {
      SCRACH_RAMBUFFER_BYTE(i) = (uint8_t)s[i];
      i++;
    }while (i <= count);
        
    SCRACH_RAMBUFFER_BYTE(count + 1) =  '\0';
    SCRACH_RAMBUFFER(256) = 0x00000000;
    SCRACH_RAMBUFFER(260) = 0x00000000;
    WRITE_TARGET_DATASLOT_ID(0) = aft->aft_dataslot;
    WRITE_TARGET_DATASLOT_BRIDGE_ADD(0) = APF_SCRACH_ADDRESS_OFFSET;
    WRITE_TARGET_DATASLOT_OFFSET(0) = APF_SCRACH_ADDRESS_OFFSET;
    WRITE_TARGET_DATASLOT_CONTROL(0) = TARGET_DATASLOT_OPEN_FILENAME_REG;
    riscusleep(10);
    strcpy(data, s);
    dataslot_pathname_file_decode(aft);

    uint32_t apf_codes;
    do
    {
      apf_codes = WRITE_TARGET_DATASLOT_CONTROL(0);
      if ((apf_codes & APF_ERROR) > 0)
      {
      // mainprintf("\033[36;5;4mAPF FAILD dataslot_write_lba? length%0.4X\033[0m\r\n", length);
        file_error_enabled = 1;
        file_error_code = error_info_dataslot_File_Not_Found;
        return (apf_codes & APF_ERROR);
      }
    // minimig_input_update();
    } while (!(apf_codes & APF_DONE));
 
    aft->aft_op_filesize  =  dataslot_size(aft->aft_dataslot);
    aft->aft_op_flags     =  SCRACH_RAMBUFFER(260);
    mainprintf("Directory %s \r\n File name %s \r\n more information %0.8x %0.8x\r\n",aft->aft_filepath, aft->aft_filename, aft->aft_op_flags, aft->aft_op_filesize);
    return 0;
}