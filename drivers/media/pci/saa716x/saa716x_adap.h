#ifndef __SAA716x_ADAP_H
#define __SAA716x_ADAP_H

struct saa716x_dev;
struct saa716x_adapter;

enum saa716x_recover_reason {
	SAA716X_REC_IRQ_ANOMALY = 0,
	SAA716X_REC_BAD_WRITE_INDEX,
};

extern int saa716x_dvb_init(struct saa716x_dev *saa716x);
extern void saa716x_dvb_exit(struct saa716x_dev *saa716x);
extern void saa716x_adapter_schedule_recovery(struct saa716x_adapter *saa716x_adap,
					      enum saa716x_recover_reason reason);

#endif /* __SAA716x_ADAP_H */
