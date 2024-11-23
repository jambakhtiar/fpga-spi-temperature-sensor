#ifndef SENSORS_H_
#define SENSORS_H_


/**
 * Default analog front-end parameters for some of the most commonly
 * used sensors along with the BoomBox
 */
 
#define ADCONV 				(10.0/32768.0)

/**
 * Imperix standard sensors
 */
#define IX_DIN50A_GAIN		(ADCONV/0.100)
#define IX_DIN800V_GAIN		(ADCONV/2.46e-3)
#define IX_DESK25A_GAIN		(ADCONV/4.93e-3)
#define IX_DESK400V_GAIN	(ADCONV/70.7e-3)


/**
 * Imperix power electronics development modules
 */
#define	IX_PEB4046_I_GAIN	(ADCONV/50.0e-3)
#define IX_PEB4046_V_GAIN	(ADCONV/5.32e-3)
#define	IX_PEB8032_I_GAIN	(ADCONV/50.0e-3)
#define IX_PEB8032_V_GAIN	(ADCONV/2.5e-3)
#define	IX_TRENCH_I_GAIN	IX_PEB4046_I_GAIN
#define IX_TRENCH_V_GAIN	IX_PEB4046_V_GAIN

#define IX_PEH2015_I_GAIN	(-ADCONV/74e-3)
#define IX_PEH2015_V_GAIN	(-ADCONV/8.9e-3)
#define IX_PEH4010_I_GAIN	(-ADCONV/74e-3)
#define IX_PEH4010_V_GAIN	(-ADCONV/8.9e-3)


/**
 * LEM voltage sensors
 */
#define LEM_LV100_400_GAIN	(ADCONV/12.5e-3)
#define LEM_LV100_500_GAIN	(ADCONV/10.0e-3)
#define LEM_LV100_600_GAIN	(ADCONV/8.33e-3)
#define LEM_LV100_800_GAIN	(ADCONV/6.67e-3)
#define LEM_LV100_1000_GAIN	(ADCONV/5.00e-3)
#define LEM_LV100_2000_GAIN	(ADCONV/1.00e-3)
#define LEM_LV200_200_GAIN	(ADCONV/40.0e-3)
#define LEM_LV200_400_GAIN	(ADCONV/20.0e-3)
#define LEM_LV200_800_GAIN	(ADCONV/10.0e-3)
#define LEM_CV3_200_GAIN	(ADCONV/41.7e-3)
#define LEM_CV3_1000_GAIN	(ADCONV/10.0e-3)
#define LEM_CV3_1200_GAIN	(ADCONV/8.33e-3)
#define LEM_CV3_1500_GAIN	(ADCONV/6.67e-3)


/**
 * LEM current sensors
 */
#define LEM_LAH25_GAIN		(ADCONV/0.100)
#define LEM_LAH50_GAIN		(ADCONV/0.050)
#define LEM_LAX100_GAIN		(ADCONV/0.050)
#define LEM_LA25_GAIN		(ADCONV/0.100)
#define LEM_LA55_GAIN		(ADCONV/0.050)
#define LEM_LA100_GAIN		(ADCONV/0.050)
#define LEM_LA125_GAIN		(ADCONV/0.050)
#define LEM_HTB50_GAIN		(ADCONV/33.3e-3)
#define LEM_HTB100_GAIN		(ADCONV/16.7e-3)
#define LEM_HTB200_GAIN		(ADCONV/8.33e-3)
#define LEM_HTB300_GAIN		(ADCONV/5.55e-3)
#define LEM_HTB400_GAIN		(ADCONV/4.17e-3)
#define LEM_HAL100_GAIN		(ADCONV/40.0e-3)
#define LEM_HAL200_GAIN		(ADCONV/20.0e-3)
#define LEM_HAL300_GAIN		(ADCONV/13.3e-3)
#define LEM_HAL400_GAIN		(ADCONV/10.0e-3)
#define LEM_HAL500_GAIN		(ADCONV/8.00e-3)
#define LEM_HAL600_GAIN		(ADCONV/6.66e-3)
#define LEM_HTA100_GAIN		(ADCONV/40.0e-3)
#define LEM_HTA200_GAIN		(ADCONV/20.0e-3)
#define LEM_HTA300_GAIN		(ADCONV/13.3e-3)
#define LEM_HTA400_GAIN		(ADCONV/10.0e-3)
#define LEM_HTA500_GAIN		(ADCONV/8.00e-3)
#define LEM_HTA600_GAIN		(ADCONV/6.66e-3)
#define LEM_HTA1000_GAIN	(ADCONV/4.00e-3)
#define LEM_HAX500_GAIN		(ADCONV/8.00e-3)
#define LEM_HAX850_GAIN		(ADCONV/4.71e-3)
#define LEM_HAX1000_GAIN	(ADCONV/4.00e-3)
#define LEM_HAX2000_GAIN	(ADCONV/2.00e-3)

#endif /* SENSORS_H_ */
