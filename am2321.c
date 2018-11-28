#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>              /* for O_RDWR */
#include <string.h>             /* for memcpy */
#include <stropts.h>            /* for ioctl */
#include <unistd.h>             /* for read/write */
#include <linux/i2c-dev.h>      /* for I2C_SLAVE */


/* I2C character device */
#define I2C_DEVICE "/dev/i2c-1"

/* I2C address of DHT12 sensor in 7 bits
 * - the first 7 bits should be passed to ioctl system call
 *   because the least 1 bit of the address represents read/write
 *   and the i2c driver takes care of it
 */
#define DHT12_ADDR 0x5C



/*
 *  st_dht12 Structure
 */

typedef struct {
  unsigned char data[8];
} st_dht12;


st_dht12 __st_dht12( unsigned char* data ) {
  st_dht12 result;
  memcpy( result.data, data, 8 );
  return result;
}

void dht12_dump( st_dht12 measured ) {
  printf( "[ 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ]\n",
          measured.data[0], measured.data[1],
          measured.data[2], measured.data[3],
          measured.data[4], measured.data[5],
          measured.data[6], measured.data[7] );
  return;
}

short __dht12_temperature( st_dht12 measured ) {
  return (measured.data[4] << 8) + measured.data[5];
}

short dht12_temperature_integral( st_dht12 measured ) {
  return measured.data[2];
}

short dht12_temperature_fraction( st_dht12 measured ) {
  return measured.data[3];
}

short __dht12_humidity( st_dht12 measured ) {
  return (measured.data[2] << 8) + measured.data[3];
}

short dht12_humidity_integral( st_dht12 measured ) {
  return measured.data[0];
}

short dht12_humidity_fraction( st_dht12 measured ) {
  return measured.data[1];
}


/*
 *  Measurement function
 */

st_dht12 dht12() {
  int fd;
  int ret;
  int retry_cnt;
  unsigned char data[8];

  /* open I2C device */
  fd = open( I2C_DEVICE, O_RDWR );
  if ( fd < 0 ) {
    perror( "dht12(1)" );
    exit( 1 );
  }

  /* set address of I2C device in 7 bits */
  ret = ioctl( fd, I2C_SLAVE, DHT12_ADDR );
  if ( ret < 0 ) {
    perror( "dht12(2)" );
    exit( 2 );
  }

  /* write measurement request */
  data[0] = 0x00;
  ret = write( fd, data, 1 );
  if ( ret < 0 ) {
    perror( "dht12(3)" );
    exit( 3 );
  }

  /* read measured result */
  memset( data, 0x00, 8 );
  ret = read( fd, data, 8 );
  if ( ret < 0 ) {
    perror( "dht12(4)" );
    exit( 4 );
  }

  /* close I2C device */
  close( fd );

  if (data[0] + data[1] + data[2] + data[3] != data[4]) {
	  fprintf(stderr, "checksum does not match");
	  exit(5);
  }

  return __st_dht12( data );
}

/*
 *  Print functions
 */

void print_dht12( st_dht12 measured ) {
  printf( "%d.%d %d.%d\n",
          dht12_temperature_integral( measured ),
          dht12_temperature_fraction( measured ),
          dht12_humidity_integral( measured ),
          dht12_humidity_fraction( measured ) );
  return;
}

void print_dht12_human_readable( st_dht12 measured ) {
  printf( "Temperature %d.%d [C]\n",
          dht12_temperature_integral( measured ),
          dht12_temperature_fraction( measured ) );
  printf( "Humidity    %d.%d [%%]\n",
          dht12_humidity_integral( measured ),
          dht12_humidity_fraction( measured ) );
  return;
}


/*
 *  Main
 */

#define OPT_HUMAN_READABLE 0x1

int print_help() {
  fprintf( stderr,
           "Usage: dht12 [-r] [-s]\n"
           "Get temperature and humidity measured with Aosong's DHT12 sensor.\n"
           "  -r    human readable output\n" );
  exit( 1 );
}

int parse_options( int argc, char* argv[]) {
  int options = 0;
  int flags = 0;
  
  while( 1+flags < argc && argv[1+flags][0] == '-' ) {
    switch( argv[1+flags][1] ) {
    case 'h': options |= OPT_HUMAN_READABLE; break;
    default:
      fprintf( stderr, "dht12: Unsupported option \"%s\"\n", argv[1+flags] );
      print_help();
      exit( 1 );
    }
    flags++;
  }
  
  return options;
}

int is_opt_human_readable( int options ) {
  return options & OPT_HUMAN_READABLE;
}

int main( int argc, char* argv[] ) {
  int options;
  st_dht12 measured;

  /* parse the given options */
  options = parse_options( argc, argv );
  
  /* measure temperature and humidity */
  measured = dht12();

  /* print measured temperature and humidity */
  if ( ! is_opt_human_readable( options ) ) {
    print_dht12( measured );
  } else {
    print_dht12_human_readable( measured );
  }
  
  return 0;
}
