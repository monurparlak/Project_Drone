#include "mpu6050.hpp"

//todo change data type to std::byte

/*
documentation
you can access a register by it's name (defined in mpu datasheet) in all lowcase
register flags are strored in anonymous struct bit fields. padded with _flags
data sheet names for register flags correspond with these bit-field variables.

ie: FIFO_EN register is defined as fifo_en
	FIFO_EN flag temp_fifo_en is modifiable via fifo_en_flags.temp_fifo_en = val


//todo documentation

note: slv0 slv1 slv2 etc. are shortened inside one flag struct slvx

*/


static constexpr std::uint8_t mpu_address = 0x68;

static constexpr std::uint8_t self_test_x = 0x0D;
static constexpr std::uint8_t self_test_y = 0x0E;
static constexpr std::uint8_t self_test_z = 0x0F;
static constexpr std::uint8_t self_test_a = 0x10;


static constexpr std::uint8_t smprt_div = 0x19;
static constexpr std::uint8_t config = 0x1A;
static constexpr std::uint8_t gyro_config = 0x1B;
static constexpr std::uint8_t accel_config = 0x1C;
static constexpr std::uint8_t mot_thr = 0x1F;
static constexpr std::uint8_t fifo_en = 0x23;


static constexpr std::uint8_t i2c_mst_ctrl = 0x24;

static constexpr std::uint8_t i2c_slv0_addr = 0x25;
static constexpr std::uint8_t i2c_slv0_reg = 0x26;
static constexpr std::uint8_t i2c_slv0_ctrl = 0x27;

static constexpr std::uint8_t i2c_slv1_addr = 0x28;
static constexpr std::uint8_t i2c_slv1_reg = 0x29;
static constexpr std::uint8_t i2c_slv1_ctrl = 0x2A;

static constexpr std::uint8_t i2c_slv2_addr = 0x2B;
static constexpr std::uint8_t i2c_slv2_reg = 0x2C;
static constexpr std::uint8_t i2c_slv2_ctrl = 0x2D;

static constexpr std::uint8_t i2c_slv3_addr = 0x2E;
static constexpr std::uint8_t i2c_slv3_reg = 0x2F;
static constexpr std::uint8_t i2c_slv3_ctrl = 0x30;

static constexpr std::uint8_t i2c_slv4_addr = 0x31;
static constexpr std::uint8_t i2c_slv4_reg = 0x32;
static constexpr std::uint8_t i2c_slv4_do = 0x32;
static constexpr std::uint8_t i2c_slv4_ctrl = 0x34;
static constexpr std::uint8_t i2c_slv4_di = 0x35;

static constexpr std::uint8_t i2c_mst_status = 0x36;


static constexpr std::uint8_t int_pin_cfg = 0x37;
static constexpr std::uint8_t int_enable = 0x38;
static constexpr std::uint8_t int_status = 0x3A;


static constexpr std::uint8_t accel_xout_h = 0x3B;
static constexpr std::uint8_t accel_xout_l = 0x3C;
static constexpr std::uint8_t accel_yout_h = 0x3D;
static constexpr std::uint8_t accel_yout_l = 0x3E;
static constexpr std::uint8_t accel_zout_h = 0x3F;
static constexpr std::uint8_t accel_zout_l = 0x40;

static constexpr std::uint8_t temp_out_h = 0x41;
static constexpr std::uint8_t temp_out_l = 0x42;

static constexpr std::uint8_t gyro_xout_h = 0x43;
static constexpr std::uint8_t gyro_xout_l = 0x44;
static constexpr std::uint8_t gyro_yout_h = 0x45;
static constexpr std::uint8_t gyro_yout_l = 0x46;
static constexpr std::uint8_t gyro_zout_h = 0x47;
static constexpr std::uint8_t gyro_zout_l = 0x48;


static constexpr std::uint8_t ext_sens_data_00 = 0x49;


static constexpr std::uint8_t i2c_slv0_do = 0x63;
static constexpr std::uint8_t i2c_slv1_do = 0x64;
static constexpr std::uint8_t i2c_slv2_do = 0x65;
static constexpr std::uint8_t i2c_slv3_do = 0x66;

static constexpr std::uint8_t i2c_mst_delay_ctrl = 0x67;


static constexpr std::uint8_t signal_path_reset = 0x68;
static constexpr std::uint8_t mot_detect_ctrl = 0x69;
static constexpr std::uint8_t user_ctrl = 0x6A;
static constexpr std::uint8_t pwr_mgmt_1 = 0x6B;
static constexpr std::uint8_t pwr_mgmt_2 = 0x6C;
static constexpr std::uint8_t fifo_count_h = 0x72;
static constexpr std::uint8_t fifo_count_l = 0x73;
static constexpr std::uint8_t fifo_r_w = 0x74;
static constexpr std::uint8_t who_am_i = 0x75;


struct
{
	std::uint8_t xyz_g_test : 5;
	std::uint8_t xyz_a_test : 3;

}static self_test_xyz_flags;


struct
{
	std::uint8_t za_g_test : 2;
	std::uint8_t ya_a_test : 2;
	std::uint8_t xa_a_test : 2;
	std::uint8_t reserved  : 2;

}static self_test_a_flags;

struct
{
	std::uint8_t smplrt_div;

}static smprt_div_flags;


struct
{
	std::uint8_t dlpf_cfg		: 3;
	std::uint8_t ext_sync_set	: 3;

}static config_flags;


struct
{
	std::uint8_t reserved	: 3;
	std::uint8_t fs_sel		: 3;
	std::uint8_t reserved2	: 2;

}static gyro_config_flags;


struct
{
	std::uint8_t reserved	: 3;
	std::uint8_t afs_sel	: 3;
	std::uint8_t xa_st		: 1;
	std::uint8_t ya_st		: 1;
	std::uint8_t za_st		: 1;

}static accel_config_flags;


struct
{
	std::uint8_t mot_thr;

}static mot_thr_flags;


struct
{
	std::uint8_t slv0_fifo_en	: 1;
	std::uint8_t slv1_fifo_en	: 1;
	std::uint8_t slv2_fifo_en	: 1;
	std::uint8_t accel_fifo_en	: 1;
	std::uint8_t zg_fifo_en		: 1;
	std::uint8_t yg_fifo_en		: 1;
	std::uint8_t xg_fifo_en		: 1;
	std::uint8_t temp_fifo_en 	: 1;

}static fifo_en_flags;


struct
{
	std::uint8_t i2c_mst_clk	: 4;
	std::uint8_t i2c_mst_p_nsr	: 1;
	std::uint8_t slv_3_fifo_en	: 1;
	std::uint8_t wait_for_es	: 1;
	std::uint8_t mult_mst_en	: 1;

}static i2c_mst_ctrl_flags;


struct
{
	std::uint8_t i2c_slvx_addr	: 7;
	std::uint8_t i2c_slvx_rw	: 1;

}static i2c_slvx_addr_flags;


struct
{
	std::uint8_t i2c_slvx_reg;

}static i2c_slvx_reg_flags;


struct
{
	std::uint8_t i2c_slvx_len		: 4;
	std::uint8_t i2c_slvx_grp		: 1;
	std::uint8_t i2c_slvx_reg_dis	: 1;
	std::uint8_t i2c_slvx_byte_sw	: 1;
	std::uint8_t i2c_slvx_en		: 1;

}static i2c_slvx_ctrl_flags;


struct
{
	std::uint8_t i2c_slvx_do;

}static i2c_slvx_do_flags;


struct
{
	std::uint8_t reserved 		: 1;
	std::uint8_t i2c_bypass_en	: 1;
	std::uint8_t fsync_int_en	: 1;
	std::uint8_t fsync_int_lvl	: 1;
	std::uint8_t int_rd_clear	: 1;
	std::uint8_t latch_int_en	: 1;
	std::uint8_t int_open		: 1;
	std::uint8_t int_level		: 1;

}static int_pin_cfg_flags;


struct
{
	std::uint8_t data_rdy_en	: 1;
	std::uint8_t reserved		: 2;
	std::uint8_t i2c_mst_en		: 1;
	std::uint8_t fifo_oflow_en	: 1;
	std::uint8_t reserved2		: 1;
	std::uint8_t mot_en			: 1;
	std::uint8_t int_open		: 1;

}static int_enable_flags;


struct
{
	std::uint8_t i2c_slv0_dly_en	: 1;
	std::uint8_t i2c_slv1_dly_en	: 1;
	std::uint8_t i2c_slv2_dly_en	: 1;
	std::uint8_t i2c_slv3_dly_en	: 1;
	std::uint8_t i2c_slv4_dly_en	: 1;
	std::uint8_t reserved			: 2;
	std::uint8_t delay_es_shadow	: 1;

}static i2c_mst_delay_ctrl_flags;


struct
{
	std::uint8_t temp_reset		: 1;
	std::uint8_t accel_reset	: 1;
	std::uint8_t gyro_reset		: 1;
	std::uint8_t reserved		: 5;

}static signal_path_reset_flags;


struct
{
	std::uint8_t reserved		: 4;
	std::uint8_t accel_on_delay	: 2;
	std::uint8_t reserved2		: 2;

}static mot_detect_ctrl_flags;


struct
{
	std::uint8_t sig_cond_reset	: 1;
	std::uint8_t i2c_mst_reset	: 1;
	std::uint8_t fifo_reset		: 1;
	std::uint8_t reserved		: 1;
	std::uint8_t i2c_if_dis		: 1;
	std::uint8_t i2c_mst_en		: 1;
	std::uint8_t fifo_en		: 1;
	std::uint8_t reserved2		: 1;

}static user_ctrl_flags;


struct
{
	std::uint8_t clk_sel		: 3;
	std::uint8_t temp_dis		: 1;
	std::uint8_t reserved		: 1;
	std::uint8_t cycle			: 1;
	std::uint8_t sleep			: 1;
	std::uint8_t device_reset	: 1;

}static pwr_mgmt_1_flags;


struct
{
	std::uint8_t stby_zg		: 1;
	std::uint8_t stby_yg		: 1;
	std::uint8_t stby_xg		: 1;
	std::uint8_t stby_za		: 1;
	std::uint8_t stby_ya		: 1;
	std::uint8_t stby_xa		: 1;
	std::uint8_t lp_wake_ctrl	: 2;

}static pwr_mgmt_2_flags;


union readout
{
	int16_t data;
	uint8_t buff[2];

	inline void swap()
	{
		uint8_t temp = buff[0];
		buff[0] = buff[1];
		buff[1] = temp;

		return;
	}
};

struct mpu6050_data
{

	readout accel_x;
	readout accel_y;
	readout accel_z;
	
	readout temperature;

	readout gyro_x;
	readout gyro_y;
	readout gyro_z;
};


readout temp = {0};

//todo: implement reading non_raw variables
bool read_raw = true;

void mpu6050_init()
{
	//use reinterpret cast to tell the compiler to stfu
	//todo: using some sort of flag mechanism would be much better, this looks quite ugly, i am thinking
	//		something like std::ios::binary

	pwr_mgmt_1_flags = {0};
	pwr_mgmt_1_flags.device_reset = true;
	i2c::write(mpu_address, pwr_mgmt_1, reinterpret_cast<uint8_t *>(&pwr_mgmt_1_flags));

	//! change this HAL_Delay(40);
	sleep_ms(40);

	pwr_mgmt_1_flags = {0};
	pwr_mgmt_1_flags.clk_sel = 1;
	i2c::write(mpu_address, pwr_mgmt_1, reinterpret_cast<uint8_t *>(&pwr_mgmt_1_flags));


	smprt_div_flags = {0};
	smprt_div_flags.smplrt_div = 7;
	i2c::write(mpu_address, smprt_div, reinterpret_cast<uint8_t *>(&smprt_div_flags));


	user_ctrl_flags = {0};
	user_ctrl_flags.fifo_en = true;
	user_ctrl_flags.fifo_reset = true;
	i2c::write(mpu_address, user_ctrl, reinterpret_cast<uint8_t *>(&user_ctrl_flags));


	fifo_en_flags = {0};
	fifo_en_flags.temp_fifo_en = true;
	fifo_en_flags.xg_fifo_en = true;
	fifo_en_flags.yg_fifo_en = true;
	fifo_en_flags.zg_fifo_en = true;
	fifo_en_flags.accel_fifo_en = true;
	i2c::write(mpu_address, fifo_en, reinterpret_cast<uint8_t *>(&fifo_en_flags));


	int_pin_cfg_flags = {0};
	int_pin_cfg_flags.latch_int_en = true;
	int_pin_cfg_flags.int_rd_clear = true;
	i2c::write(mpu_address, int_pin_cfg, reinterpret_cast<uint8_t *>(&int_pin_cfg_flags));


	int_enable_flags = {0};
	int_enable_flags.fifo_oflow_en = true;
	i2c::write(mpu_address, int_enable, reinterpret_cast<uint8_t *>(&int_enable_flags));


	return;
}

//keep in mind that mcu uses little-endian format while 6050 uses big-endian

int16_t read_accel_x(void)
{
	i2c::read(mpu_address, accel_xout_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

int16_t read_accel_y(void)
{
	i2c::read(mpu_address, accel_yout_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

int16_t read_accel_z(void)
{
	i2c::read(mpu_address, accel_xout_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

void read_accel(int16_t *buffer)
{
	readout *swp = reinterpret_cast<readout *>(buffer);

	i2c::read(mpu_address, accel_xout_h, swp->buff, 6);

	for(std::size_t i = 0; i < 3; ++i)
	swp[i].swap();

	return;
}

int16_t read_gyro_x(void)
{
	i2c::read(mpu_address, accel_xout_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

int16_t read_gyro_y(void)
{
	i2c::read(mpu_address, gyro_yout_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

int16_t read_gyro_z(void)
{
	i2c::read(mpu_address, gyro_yout_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

void read_gyro(int16_t *buffer)
{
	readout *swp = reinterpret_cast<readout *>(buffer);

	i2c::read(mpu_address, gyro_xout_h, swp->buff, 6);

	for(std::size_t i = 0; i < 3; ++i)
	swp[i].swap();

	return;
}

int16_t read_temp(void)
{
	i2c::read(mpu_address, temp_out_h, temp.buff, 2);
	temp.swap();

	return temp.data;
}

void read_all(int16_t *buffer)
{
	readout *swp = reinterpret_cast<readout *>(buffer);

	i2c::read(mpu_address, accel_xout_h, swp->buff, 14);

	for(std::size_t i = 0; i < 7; ++i)
	swp[i].swap();

	return;
}

void read_fifo(uint8_t *buffer, std::size_t size)
{

	if(size > 1024)	size = 1024;

	i2c::read(mpu_address, fifo_r_w, buffer, size);

	return;
}

int16_t fifo_size()
{
	i2c::read(mpu_address, fifo_count_h, temp.buff, 2);
	temp.swap();
	
	return temp.data;
}

void reset_fifo()
{
	user_ctrl_flags = {0};
	user_ctrl_flags.fifo_reset = true;

	i2c::write(mpu_address, user_ctrl, reinterpret_cast<uint8_t *>(&user_ctrl_flags));

	return;
}


void enable_fifo()
{
	user_ctrl_flags = {0};
	user_ctrl_flags.fifo_en = true;
	user_ctrl_flags.fifo_reset = true;

	i2c::write(mpu_address, user_ctrl, reinterpret_cast<uint8_t *>(&user_ctrl_flags));

	return;
}

void reset()
{
	pwr_mgmt_1_flags = {0};
	pwr_mgmt_1_flags.device_reset = true;

	i2c::write(mpu_address, pwr_mgmt_1, reinterpret_cast<uint8_t *>(&pwr_mgmt_1_flags));

	return;
}

uint8_t read_interrupt()
{
	i2c::read(mpu_address, int_status, temp.buff, 1);
	
	return static_cast<uint8_t>(temp.buff[0]);
}