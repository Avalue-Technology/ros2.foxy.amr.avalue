#ifndef YESENSE_STD_OUT_DECODER_H
#define YESENSE_STD_OUT_DECODER_H

namespace yesense
{

#pragma pack(1)

typedef struct
{
	float x;
	float y;
	float z;
}axis_data_t;

typedef struct
{
	float pitch;
	float roll;
	float yaw;
}euler_data_t;

typedef struct
{
	float q0;	
	float q1;
	float q2;
	float q3;
}quat_data_t;

typedef struct
{
	double latitude;
	double longitude;
	float altitude;
}pos_data_t;

typedef struct
{
	float vel_e;
	float vel_n;
	float vel_u;
}vel_data_t;

typedef struct
{
	unsigned short  year;
	unsigned int    month  :4;
	unsigned int    day    :5;
	unsigned int    hour   :5;
	unsigned int    min    :6;
	unsigned int    sec    :6;
	unsigned int    resv   :6;
	unsigned short  ms;
}utc_data_t;

typedef struct
{
	union 
	{
		struct
		{
			unsigned char fusion_sta:4; 
			unsigned char pos_sta   :4;
		}bit;
		unsigned char byte;
	}status;
}nav_status_t;

typedef struct
{
	unsigned short valid_flg            :1;
	unsigned short sensor_temp          :1;
	unsigned short acc                  :1;   
	unsigned short gyro                 :1;
	unsigned short mag_norm             :1;
	unsigned short mag_raw              :1;
	unsigned short euler                :1;
	unsigned short quat                 :1;

	unsigned short pos                  :1;
	unsigned short utc                  :1;      
	unsigned short vel                  :1;
	unsigned short status               :1;
	unsigned short sample_timestamp     :1;          
	unsigned short dataready_timestamp  :1;  
	unsigned short pressure             :1;
	unsigned short resv                 :1;                    
}yis_content_t;

typedef struct
{
	yis_content_t   content;         
	unsigned short  tid;

	float           sensor_temp;
	axis_data_t     acc;
	axis_data_t     gyro;
	axis_data_t     mag_raw;
	axis_data_t     mag_norm;
	float           pressure;

	euler_data_t    euler;
	quat_data_t     quat;

	pos_data_t      pos;
	vel_data_t      vel;
	utc_data_t      utc;
	unsigned int    sample_timestamp;
	unsigned int    dataready_timestamp;

	nav_status_t    status;
}yis_out_data_t;

typedef struct
{
	unsigned int st_idx;
	unsigned int end_idx;
}msg_idx_t;
#pragma pack()

class yis_std_out_decoder
{
public:
	yis_std_out_decoder();
	~yis_std_out_decoder();

	int crc_calc(unsigned char *data, unsigned int len, unsigned short *crc);
	int data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result);
	msg_idx_t *msg_idx_obt(void);

private:

#pragma pack(1)
	typedef struct
	{
		unsigned char   header1;
		unsigned char   header2;
		unsigned short  tid;
		unsigned char   len;
	}output_data_header_t;

	typedef struct
	{
		unsigned char data_id;
		unsigned char data_len;
	}payload_info_t;
#pragma pack()

	msg_idx_t msg_idx;

	int convert_data_s32(unsigned char *data, float *result, float sens, unsigned short cnt);
	int parse_data_by_id(payload_info_t *info, unsigned char *data, yis_out_data_t *result);
};

}

#endif
