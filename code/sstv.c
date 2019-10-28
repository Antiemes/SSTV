#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>

//Atomic unit: 57.2 us
/*1 line:
  85 1200
  10 1500
  1280 green
  10 1500
  1280 blue
  10 1500
  1280 red
  10 1500
*/

//Frequency = 1500 + (ColorByte * 3.1372549)

#define SINTABLE_LENGTH 1024
//#define SRATE 17483
#define SRATE 17438

uint8_t sintable[1024];
int fd;
uint16_t phase=0;

uint16_t lumToFreq(uint8_t lum)
{
  return 1500 + (lum * 3.1372549);
}

void makeSample(uint16_t len, uint16_t freq)
{
  uint8_t sample;
  uint16_t i;
  
  for (i=0; i<len; i++)
  {
    phase+=64*freq/17;
    sample=sintable[(phase>>6)&(SINTABLE_LENGTH-1)];
    write(fd, &sample, 1);
  }
}

int main(int argc, char** argv)
{
  int i;
  int y;
  int fd_in;

  fd=open("ki.raw", O_WRONLY | O_CREAT | O_TRUNC, 0666);

  for (i=0; i<SINTABLE_LENGTH; i++)
  {
    sintable[i]=(sin(2.0*M_PI*i/SINTABLE_LENGTH)+1)*127.5;
  }
  
  makeSample(SRATE*300/1000, 1900);
  makeSample(SRATE* 10/1000, 1200);
  makeSample(SRATE*300/1000, 1900);
  makeSample(SRATE* 30/1000, 1200);
  
  makeSample(SRATE* 30/1000, 1300); // b0
  makeSample(SRATE* 30/1000, 1300); // b1
  makeSample(SRATE* 30/1000, 1300); // b2
  makeSample(SRATE* 30/1000, 1100); // b3
  makeSample(SRATE* 30/1000, 1300); // b4
  makeSample(SRATE* 30/1000, 1100); // b5
  makeSample(SRATE* 30/1000, 1300); // b6
  
  makeSample(SRATE* 30/1000, 1300); // parity
  
  makeSample(SRATE* 30/1000, 1200);
  
  fd_in=open(argv[1], O_RDONLY);
  for (y=0; y<256; y++)
  {
    uint8_t buffer[320*3];

    read(fd_in, buffer, 320*3);

    makeSample(85, 1200);
    makeSample(10, 1500);
    for (i=0; i<320; i++)
    {
      makeSample(4, lumToFreq(buffer[i*3+1]));
    }
    makeSample(10, 1500);
    for (i=0; i<320; i++)
    {
      makeSample(4, lumToFreq(buffer[i*3+2]));
    }
    makeSample(10, 1500);
    for (i=0; i<320; i++)
    {
      makeSample(4, lumToFreq(buffer[i*3  ]));
    }
  }

  close(fd_in);
  close(fd);

  char* command;
  asprintf(&command, "sox -r %d -e unsigned -b 8 -c 1 ki.raw ki.wav", SRATE);
  system(command);
  free(command);

  return 0;
}

