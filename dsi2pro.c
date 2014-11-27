
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>

#include "libusb.h"
#include "fitsio.h"

// Endpoints
#define EP_IN 0x01
#define EP_OUT 0x81
#define EP_DATA 0x86

// ACK & NACK
#define RES_ACK 0x06
#define RES_NAK 0x15

// Commands
#define CMD_PING 0x00
#define CMD_RESET 0x01
#define CMD_ABORT 0x02
#define CMD_TRIGGER 0x03
#define CMD_CLEAR_TS 0x04

#define CMD_GET_VERSION 0x14
#define CMD_GET_STATUS 0x15
#define CMD_GET_TIMESTAMP 0x16
#define CMD_GET_EEPROM_LENGTH 0x1e
#define CMD_GET_EEPROM_BYTE 0x1f

#define CMD_SET_EEPROM_BYTE 0x20

#define CMD_GET_GAIN 0x32                        //
#define CMD_SET_GAIN 0x33                        //
#define CMD_GET_OFFSET 0x34                      //
#define CMD_SET_OFFSET 0x35                      //
#define CMD_GET_EXP_TIME 0x36                    //
#define CMD_SET_EXP_TIME 0x37                    //
#define CMD_GET_EXP_MODE 0x38                    //
#define CMD_SET_EXP_MODE 0x39                    //
#define CMD_GET_VDD_MODE 0x3a                    //
#define CMD_SET_VDD_MODE 0x3b                    //
#define CMD_GET_FLUSH_MODE 0x3c                  //
#define CMD_SET_FLUSH_MODE 0x3d                  //
#define CMD_GET_CLEAN_MODE 0x3e                  //
#define CMD_SET_CLEAN_MODE 0x3f                  //
#define CMD_GET_READOUT_SPD 0x40                 //
#define CMD_SET_READOUT_SPD 0x41                 //
#define CMD_GET_READOUT_MODE 0x42                //
#define CMD_SET_READOUT_MODE 0x43                //
#define CMD_GET_NORM_READOUT_DELAY 0x44          //
#define CMD_SET_NORM_READOUT_DELAY 0x45          //
#define CMD_GET_ROW_COUNT_ODD 0x46               //
#define CMD_SET_ROW_COUNT_ODD 0x47               //
#define CMD_GET_ROW_COUNT_EVEN 0x48              //
#define CMD_SET_ROW_COUNT_EVEN 0x49              //
#define CMD_GET_TEMP 0x4a                        //
#define CMD_GET_EXP_TIMER_COUNT 0x4b
#define CMD_PS_ON 0x64                           //
#define CMD_PS_OFF 0x65                          //
#define CMD_CCD_VDD_ON 0x66                      //
#define CMD_CCD_VDD_OFF 0x67                     //
#define CMD_AD_READ 0x68
#define CMD_AD_WRITE 0x69
#define CMD_TEST_PATTERN 0x6a
#define CMD_GET_DEBUG_VALUE 0x6b
#define CMD_GET_EEPROM_VIDPID 0x6c
#define CMD_SET_EEPROM_VIDPID 0x6d
#define CMD_ERASE_EEPROM 0x6e


// Parameters values
#define VDD_MODE_AUTOMATIC 0x00
#define VDD_MODE_ALWAYS_ON 0x01
#define VDD_MODE_ALWAYS_OFF 0x02

#define READOUT_MODE_DUAL_EXPOSURE 0x00
#define READOUT_MODE_SINGLE_EXPOSURE 0x01
#define READOUT_MODE_ODD_FIELDS 0x02
#define READOUT_MODE_EVEN_FIELDS 0x03

#define FLUSH_MODE_CONTINUOUS 0x00
#define FLUSH_MODE_BEFORE_EXPOSURE 0x01
#define FLUSH_MODE_BEFORE_NEVER 0x02

#define EXPOSURE_MODE_SINGLE 0x00
#define EXPOSURE_MODE_CONTINUOUS 0x01

#define CLEAN_MODE_ENABLED 0x00
#define CLEAN_MODE_DISABLED 0x01

#define READOUT_SPEED_NORMAL 0x00
#define READOUT_SPEED_HIGH 0x01



// HW properties
#define IMG_WIDTH 752
#define IMG_HEIGHT 582
#define IMG_EVEN 299
#define IMG_ODD 298
#define IMG_CHUNK 2048
#define IMG_CHUKN_EVEN (IMG_CHUNK*IMG_EVEN)
#define IMG_CHUKN_ODD (IMG_CHUNK*IMG_ODD)

/* ========================================================================= */

static struct libusb_device_handle *rc_dev_dsi = NULL;
static int seq = 1;
static unsigned int VID = 0x156C;
static unsigned int PID = 0x0101;
static unsigned char rawA[IMG_CHUKN_EVEN];
static unsigned char rawB[IMG_CHUKN_ODD];
static unsigned char raw[IMG_WIDTH * IMG_HEIGHT * 2];

/**
 * Find Device
 **/
static int rc_find_device() {
	rc_dev_dsi = libusb_open_device_with_vid_pid(NULL, VID, PID);
	return rc_dev_dsi ? 0 : -EIO;
}


/**
 * Open
 **/
static int rc_open() {
	int r;
	r = libusb_init(NULL);
	if (r < 0) return r;

	// debug	
	libusb_set_debug(NULL,3);

	// search
	r = rc_find_device();
	if (r < 0) {
		fprintf(stderr, "rc_open error: unable find device\n");
		return r;
	}
	
	// claim
	r = libusb_claim_interface(rc_dev_dsi, 0);
	if (r < 0) {
		fprintf(stderr, "rc_open error: unable claim interface\n");
		return r;	
	}
	return 0;
}

/**
 * Close
 **/
static int rc_close() {
	// close
	libusb_close(rc_dev_dsi);
	// exit	
	libusb_exit(NULL);
}

/**
 * Get Image response
 **/
static int rc_responseImage(unsigned char *data, int len) {
	int r, t;

	// transfer
	r = libusb_bulk_transfer(rc_dev_dsi, EP_DATA, data, len, &t, 0);
	
	// result	
	if (r < 0) {
		printf("rc_responseImage error: %d\n", r);
		return r;	
	}
	return t;
}

/**
 * Priitive cmd
 **/
static int cmd(int cmd, int arg, int in, int out) {
	unsigned char data[7];
	int r, t;
        // rq
        switch (in) {
            case 0:
                data[0] = 3;
                data[1] = seq++;
                data[2] = cmd;
		r = libusb_bulk_transfer(rc_dev_dsi, EP_IN, data, 3, &t, 0);
                break;
            case 1:
                data[0] = 4;
                data[1] = seq++;
                data[2] = cmd;
                data[3] = arg;
		r = libusb_bulk_transfer(rc_dev_dsi, EP_IN, data, 4, &t, 0);
                break;
            case 2:
                data[0] = 5;
                data[1] = seq++;
                data[2] = cmd;
                data[3] = arg;
                data[4] = (arg >> 8);
		r = libusb_bulk_transfer(rc_dev_dsi, EP_IN, data, 5, &t, 0);
                break;
            case 4:
                data[0] = 7;
                data[1] = seq++;
                data[2] = cmd;
                data[3] = arg;
                data[4] = (arg >> 8);
                data[5] = (arg >> 16);
                data[6] = (arg >> 24);
		r = libusb_bulk_transfer(rc_dev_dsi, EP_IN, data, 7, &t, 0);
                break;
            default:
                printf("Unknown IN: %d\n", in);
                return 0;
        }
        // resp
        switch(out) {
            case 0:
		r = libusb_bulk_transfer(rc_dev_dsi, EP_OUT, data, 3, &t, 0);
                return data[2];
            case 1:
		r = libusb_bulk_transfer(rc_dev_dsi, EP_OUT, data, 4, &t, 0);
                return data[3] & 0xFF;
            case 2:
		r = libusb_bulk_transfer(rc_dev_dsi, EP_OUT, data, 5, &t, 0);
                return (data[3] & 0xFF) + ((data[4] & 0xFF) << 8);
            case 4:
               	r = libusb_bulk_transfer(rc_dev_dsi, EP_OUT, data, 7, &t, 0);
		return (data[3] & 0xFF) + ((data[4] & 0xFF) << 8) + ((data[5] & 0xFF) << 16) + ((data[6] & 0xFF) << 24);
            default:
                printf("Unknown OUT: %d\n", out);
                return 0;
        }
}


static int RESET() {
        return cmd(CMD_RESET, 0, 0, 0) == RES_ACK;
}

static int SET_GAIN(int value) {
        return cmd(CMD_SET_GAIN, value, 1, 0) == RES_ACK;
}

static int SET_OFFSET(int value) {
        return cmd(CMD_SET_OFFSET, value, 2, 0) == RES_ACK;
}

static int SET_ROW_COUNT_EVEN(int value) {
        return cmd(CMD_SET_ROW_COUNT_EVEN, value, 2, 0) == RES_ACK;
}

static int SET_ROW_COUNT_ODD(int value) {
        return cmd(CMD_SET_ROW_COUNT_ODD, value, 2, 0) == RES_ACK;
}

static int SET_VDD_MODE(int value) {
        return cmd(CMD_SET_VDD_MODE, value, 1, 0) == RES_ACK;
}

static int SET_FLUSH_MODE(int value) {
        return cmd(CMD_SET_FLUSH_MODE, value, 1, 0) == RES_ACK;
}

static int SET_CLEAN_MODE(int value) {
        return cmd(CMD_SET_CLEAN_MODE, value, 1, 0) == RES_ACK;
}

static int SET_READOUT_MODE(int value) {
        return cmd(CMD_SET_READOUT_MODE, value, 1, 0) == RES_ACK;
}

static int SET_READOUT_SPD(int value) {
        return cmd(CMD_SET_READOUT_SPD, value, 1, 0) == RES_ACK;
}

static int SET_EXP_TIME(int value) {
        return cmd(CMD_SET_EXP_TIME, value, 4, 0) == RES_ACK;
}

static int SET_EXP_MODE(int value) {
        return cmd(CMD_SET_EXP_MODE, value, 1, 0) == RES_ACK;
}
   
static int SET_NORM_READOUT_DELAY(int value) {
        return cmd(CMD_SET_NORM_READOUT_DELAY, value, 2, 0) == RES_ACK;
}

static int TRIGGER(int test, int exp) {
	int i, j, k, l;

	printf("Expose: %d in mode %d\n", exp, test);

        if (cmd((test) ? CMD_TEST_PATTERN : CMD_TRIGGER, 0, 0, 0) != RES_ACK) return 0;
	
	// pause if not a test
	if (test == 0) usleep(exp * 100);

	// get image by two bilk
	rc_responseImage(rawA, IMG_CHUKN_EVEN);
	rc_responseImage(rawB, IMG_CHUKN_ODD);

        // merge interleave blocks
        for (i = 5, j = 0, k = 5 * IMG_CHUNK + 58; i < 296; i++, j += IMG_WIDTH * 4, k += IMG_CHUNK) {
            //System.arraycopy(rawA, k, raw, j, IMG_WIDTH * 2);
		for (l = 0; l < IMG_WIDTH * 2; l++) raw[j + l] = rawA[k + l];	
        }
        for (i = 5, j = IMG_WIDTH * 2, k = 5 * IMG_CHUNK + 58; i < 296; i++, j += IMG_WIDTH * 4, k += IMG_CHUNK) { 
	    //System.arraycopy(rawB, k, raw, j, IMG_WIDTH * 2);
		for (l = 0; l < IMG_WIDTH * 2; l++) raw[j + l] = rawB[k + l];	
        }

	// swap bytes
	for (i = 0; i < IMG_WIDTH * IMG_HEIGHT * 2; i += 2) {
		k = raw[i];
		raw[i] = raw[i+1];
		raw[i+1] = k;
	} 

	return 1;
}


/**
 * Default Settings
 **/
static void rc_defaults() {
        RESET();
//        SET_GAIN(0);
//        SET_OFFSET(300);
        SET_ROW_COUNT_EVEN(IMG_EVEN);
        SET_ROW_COUNT_ODD(IMG_ODD);
        SET_VDD_MODE(1);
        SET_FLUSH_MODE(0);
        SET_CLEAN_MODE(0);
        SET_READOUT_MODE(0);
        SET_READOUT_SPD(1);
        SET_EXP_MODE(1);
//        SET_EXP_TIME(50);
        SET_NORM_READOUT_DELAY(1);
}

/**
 * Write buffer to RAW file
 **/
void rc_write(unsigned char *img, int len, const char *basename)
{
	FILE *fo;
	fo = fopen (basename, "w");
	fwrite(img, len, len, fo);
	fclose(fo);
}

/**
 * Write buffer to FITS file
 **/
void rc_fits(int exp)
{
	fitsfile *fptr;
	/* pointer to the FITS file; defined in fitsio.h */
	int status, ii, jj;
	long fpixel = 1, naxis = 2, nelements;
	float exposure;
	long naxes[2] = { IMG_WIDTH, IMG_HEIGHT };
	status = 0;
	/* initialize status before calling fitsio routines */
	fits_create_file(&fptr, "testfile.fits", &status);
	fits_create_img(fptr, SHORT_IMG, naxis, naxes, &status);
	exposure = exp * 0.0001;
	fits_update_key(fptr, TFLOAT, "EXPOSURE", &exposure, "Total Exposure Time", &status);
	nelements = naxes[0] * naxes[1];
	fits_write_img(fptr, TSHORT, fpixel, nelements, &raw[0], &status);
	fits_close_file(fptr, &status);
	fits_report_error(stderr, status);
}


/* ========================================================================= */

int main(int argc, char **argv)
{
	// vars
	int r, t, i;

	// usage
	if (argc == 1) {
		printf("usage:\n");
		printf(" dsi2pro -e exposure -g gain -o offset -t -r -f\n");
		printf("  exposure is count of 1/10000 sec.\n");
		printf("  gain is value from 0 to 63\n");
		printf("  offset is value from 0 to 1023\n");
		printf("  t flag is TEST mode\n");
		printf("  r flag is RAW mode (little-endian) of output file\n");
		printf("  f flag is FITS mode of output file\n");
		return 0;
	}

	// parse opts	
	char *opts = "e:g:o:trf"; // options
	int exposure = -1; // exposure
	int gain = -1; // gain
	int offset = -1; // offset
	int test = 0; // test
	int raw = 0; // raw
	int fits = 0; // raw

	int opt;
	while((opt = getopt(argc, argv, opts)) != -1) { // call getopt while != -1
		switch(opt) {
			case 'e': // option -x, exposure
				exposure = atoi(optarg);
				break;
			case 'g': // option -g, gain
				gain = atoi(optarg);
				break;
			case 'o': // option -o, offset
				offset = atoi(optarg);
				break;
			case 't': // test
				test = 1;
				break;
			case 'r': // raw
				raw = 1;
				break;
			case 'f': // fits
				fits = 1;
				break;
		}
	}

	// postcheck
	if (exposure != -1) {
		if (exposure < 1) exposure = 1;
		if (exposure > 36000000) exposure = 36000000;
	}
	if (gain != -1) {
		if (gain < 0) gain = 0;
		if (gain > 63) gain = 63;
	}
	if (offset != -1) {
		if (offset < 0) offset = 0;
		if (offset > 1023) offset = 1023;
	}

	// open device
	if (rc_open() == 0) {

		// defaults
		rc_defaults();

		// gain
		if (gain != -1) SET_GAIN(gain);
		// offset
		if (offset != -1) SET_OFFSET(offset);
		// exposure
		if (exposure != -1) SET_EXP_TIME(exposure);

		// trigger
		TRIGGER(test, exposure);

		// close
		rc_close();

		// write FITS file
		if (fits) {
			rc_fits(exposure);
		}	

		// write RAW file
		if (raw) {
			rc_write(raw, IMG_WIDTH * IMG_HEIGHT * 2, "test.raw");
		}
	}

}

