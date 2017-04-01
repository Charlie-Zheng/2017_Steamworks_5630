package org.usfirst.frc.team5630.robot;

public class MiddleGearProfile {
	public static final int kNumPoints =143;		
	// Position (rotations)	Velocity (RPM)	Duration (ms)
	public static double [][]Points = new double[][]{		
	{0,	0	,20},
	{9.75609756097561E-05,	0.585365854	,20},
	{0.000439024390243903,	1.463414634	,20},
	{0.0011219512195122,	2.634146341	,20},
	{0.00224390243902439,	4.097560976	,20},
	{0.00390243902439024,	5.853658537	,20},
	{0.00619512195121951,	7.902439024	,20},
	{0.00921951219512195,	10.24390244	,20},
	{0.0130731707317073,	12.87804878	,20},
	{0.0178536585365854,	15.80487805	,20},
	{0.0236585365853659,	19.02439024	,20},
	{0.0304878048780488,	21.95121951	,20},
	{0.0382926829268293,	24.87804878	,20},
	{0.0470731707317073,	27.80487805	,20},
	{0.0568292682926829,	30.73170732	,20},
	{0.0675609756097561,	33.65853659	,20},
	{0.0792682926829268,	36.58536585	,20},
	{0.0919512195121951,	39.51219512	,20},
	{0.105609756097561,	42.43902439	,20},
	{0.120243902439024,	45.36585366	,20},
	{0.135853658536585,	48.29268293	,20},
	{0.152439024390244,	51.2195122	,20},
	{0.17,	54.14634146	,20},
	{0.188536585365854,	57.07317073	,20},
	{0.208048780487805,	60	,20},
	{0.228536585365854,	62.92682927	,20},
	{0.25,	65.85365854	,20},
	{0.272439024390244,	68.7804878	,20},
	{0.295853658536585,	71.70731707	,20},
	{0.320243902439024,	74.63414634	,20},
	{0.345609756097561,	77.56097561	,20},
	{0.371951219512195,	80.48780488	,20},
	{0.399268292682927,	83.41463415	,20},
	{0.427560975609756,	86.34146341	,20},
	{0.456829268292683,	89.26829268	,20},
	{0.487073170731708,	92.19512195	,20},
	{0.51829268292683,	95.12195122	,20},
	{0.550487804878049,	98.04878049	,20},
	{0.583658536585366,	100.9756098	,20},
	{0.617804878048781,	103.902439	,20},
	{0.652926829268293,	106.8292683	,20},
	{0.688975609756098,	109.4634146	,20},
	{0.725853658536586,	111.804878	,20},
	{0.763463414634147,	113.8536585	,20},
	{0.801707317073171,	115.6097561	,20},
	{0.840487804878049,	117.0731707	,20},
	{0.879707317073171,	118.2439024	,20},
	{0.919268292682927,	119.1219512	,20},
	{0.959073170731708,	119.7073171	,20},
	{0.999024390243903,	120	,20},
	{1.0390243902439,	120	,20},
	{1.0790243902439,	120	,20},
	{1.1190243902439,	120	,20},
	{1.1590243902439,	120	,20},
	{1.1990243902439,	120	,20},
	{1.2390243902439,	120	,20},
	{1.2790243902439,	120	,20},
	{1.3190243902439,	120	,20},
	{1.3590243902439,	120	,20},
	{1.3990243902439,	120	,20},
	{1.4390243902439,	120	,20},
	{1.4790243902439,	120	,20},
	{1.5190243902439,	120	,20},
	{1.5590243902439,	120	,20},
	{1.5990243902439,	120	,20},
	{1.6390243902439,	120	,20},
	{1.6790243902439,	120	,20},
	{1.7190243902439,	120	,20},
	{1.7590243902439,	120	,20},
	{1.7990243902439,	120	,20},
	{1.8390243902439,	120	,20},
	{1.8790243902439,	120	,20},
	{1.9190243902439,	120	,20},
	{1.9590243902439,	120	,20},
	{1.9990243902439,	120	,20},
	{2.0390243902439,	120	,20},
	{2.0790243902439,	120	,20},
	{2.1190243902439,	120	,20},
	{2.1590243902439,	120	,20},
	{2.1990243902439,	120	,20},
	{2.2390243902439,	120	,20},
	{2.2790243902439,	120	,20},
	{2.3190243902439,	120	,20},
	{2.3590243902439,	120	,20},
	{2.3990243902439,	120	,20},
	{2.4390243902439,	120	,20},
	{2.4790243902439,	120	,20},
	{2.5190243902439,	120	,20},
	{2.5590243902439,	120	,20},
	{2.5990243902439,	120	,20},
	{2.6390243902439,	120	,20},
	{2.6790243902439,	120	,20},
	{2.7190243902439,	120	,20},
	{2.75892682926829,	119.4146341	,20},
	{2.79858536585366,	118.5365854	,20},
	{2.83790243902439,	117.3658537	,20},
	{2.87678048780488,	115.902439	,20},
	{2.91512195121951,	114.1463415	,20},
	{2.95282926829268,	112.097561	,20},
	{2.98980487804878,	109.7560976	,20},
	{3.0259512195122,	107.1219512	,20},
	{3.06117073170732,	104.195122	,20},
	{3.09536585365854,	100.9756098	,20},
	{3.12853658536585,	98.04878049	,20},
	{3.16073170731707,	95.12195122	,20},
	{3.1919512195122,	92.19512195	,20},
	{3.22219512195122,	89.26829268	,20},
	{3.25146341463415,	86.34146341	,20},
	{3.27975609756098,	83.41463415	,20},
	{3.30707317073171,	80.48780488	,20},
	{3.33341463414634,	77.56097561	,20},
	{3.35878048780488,	74.63414634	,20},
	{3.38317073170732,	71.70731707	,20},
	{3.40658536585366,	68.7804878	,20},
	{3.4290243902439,	65.85365854	,20},
	{3.45048780487805,	62.92682927	,20},
	{3.4709756097561,	60	,20},
	{3.49048780487805,	57.07317073	,20},
	{3.5090243902439,	54.14634146	,20},
	{3.52658536585366,	51.2195122	,20},
	{3.54317073170732,	48.29268293	,20},
	{3.55878048780488,	45.36585366	,20},
	{3.57341463414634,	42.43902439	,20},
	{3.58707317073171,	39.51219512	,20},
	{3.59975609756098,	36.58536585	,20},
	{3.61146341463415,	33.65853659	,20},
	{3.62219512195122,	30.73170732	,20},
	{3.6319512195122,	27.80487805	,20},
	{3.64073170731707,	24.87804878	,20},
	{3.64853658536586,	21.95121951	,20},
	{3.65536585365854,	19.02439024	,20},
	{3.66121951219512,	16.09756098	,20},
	{3.66609756097561,	13.17073171	,20},
	{3.67004878048781,	10.53658537	,20},
	{3.67317073170732,	8.195121951	,20},
	{3.67556097560976,	6.146341463	,20},
	{3.67731707317073,	4.390243902	,20},
	{3.67853658536585,	2.926829268	,20},
	{3.67931707317073,	1.756097561	,20},
	{3.67975609756098,	0.87804878	,20},
	{3.6799512195122,	0.292682927	,20},
	{3.68,	4.06179E-16	,20},
	{3.68,	0	,20}};



}