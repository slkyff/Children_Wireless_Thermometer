
#ifndef USER_DRV_AD7171OP_H
#define USER_DRV_AD7171OP_H

enum
{
	EN_AD7171_NORMAL_CLT_MODE = 1,			 
	EN_AD7171_CALI_CONSTANT_RES_MODE	 		
};

extern void ad7171_start_acquisite(unsigned int acquisite_interval,unsigned char* ad_mode);
extern void ad7171_stop_acquisite(void);
extern unsigned int ad7171_acquisite_interval_get(void);
extern void ad7171_para_init(void);
extern void ad7171_res_init(void);

extern unsigned char ad7171_get_cur_temp_state(void);


#endif

