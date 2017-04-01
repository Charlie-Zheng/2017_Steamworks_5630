package org.usfirst.frc.team5630.robot;

public class RightGearProfile {
	public static final int kNumPoints =192;		
	// Position (rotations)	Velocity (RPM)	Duration (ms)
	public static double [][]Points = new double[][]{		
	{0,	0	,20},
	{0.000028169014084507,	0.169014085	,20},
	{0.000126760563380282,	0.422535211	,20},
	{0.000323943661971831,	0.76056338	,20},
	{0.000647887323943662,	1.183098592	,20},
	{0.00112676056338028,	1.690140845	,20},
	{0.0017887323943662,	2.281690141	,20},
	{0.00266197183098592,	2.957746479	,20},
	{0.00377464788732394,	3.718309859	,20},
	{0.00515492957746479,	4.563380282	,20},
	{0.00683098591549296,	5.492957746	,20},
	{0.00883098591549296,	6.507042254	,20},
	{0.0111830985915493,	7.605633803	,20},
	{0.0139154929577465,	8.788732394	,20},
	{0.017056338028169,	10.05633803	,20},
	{0.0206338028169014,	11.4084507	,20},
	{0.0246760563380282,	12.84507042	,20},
	{0.0292112676056338,	14.36619718	,20},
	{0.0342676056338028,	15.97183099	,20},
	{0.0398732394366197,	17.66197183	,20},
	{0.046056338028169,	19.43661972	,20},
	{0.0528169014084507,	21.12676056	,20},
	{0.0601408450704225,	22.81690141	,20},
	{0.0680281690140845,	24.50704225	,20},
	{0.0764788732394366,	26.1971831	,20},
	{0.0854929577464789,	27.88732394	,20},
	{0.0950704225352113,	29.57746479	,20},
	{0.105211267605634,	31.26760563	,20},
	{0.115915492957746,	32.95774648	,20},
	{0.127183098591549,	34.64788732	,20},
	{0.139014084507042,	36.33802817	,20},
	{0.151408450704225,	38.02816901	,20},
	{0.164366197183099,	39.71830986	,20},
	{0.177887323943662,	41.4084507	,20},
	{0.191971830985915,	43.09859155	,20},
	{0.206619718309859,	44.78873239	,20},
	{0.221830985915493,	46.47887324	,20},
	{0.237605633802817,	48.16901408	,20},
	{0.253943661971831,	49.85915493	,20},
	{0.270845070422535,	51.54929577	,20},
	{0.28830985915493,	53.23943662	,20},
	{0.306338028169014,	54.92957746	,20},
	{0.324929577464789,	56.61971831	,20},
	{0.344084507042253,	58.30985915	,20},
	{0.363802816901408,	60	,20},
	{0.384084507042253,	61.69014085	,20},
	{0.404929577464789,	63.38028169	,20},
	{0.426338028169014,	65.07042254	,20},
	{0.448309859154929,	66.76056338	,20},
	{0.470845070422535,	68.45070423	,20},
	{0.493943661971831,	70.14084507	,20},
	{0.517605633802817,	71.83098592	,20},
	{0.541830985915493,	73.52112676	,20},
	{0.566619718309859,	75.21126761	,20},
	{0.591971830985915,	76.90140845	,20},
	{0.617887323943662,	78.5915493	,20},
	{0.644366197183099,	80.28169014	,20},
	{0.671408450704225,	81.97183099	,20},
	{0.699014084507042,	83.66197183	,20},
	{0.727183098591549,	85.35211268	,20},
	{0.755915492957747,	87.04225352	,20},
	{0.785211267605634,	88.73239437	,20},
	{0.815070422535211,	90.42253521	,20},
	{0.845492957746479,	92.11267606	,20},
	{0.876478873239437,	93.8028169	,20},
	{0.908028169014085,	95.49295775	,20},
	{0.940140845070423,	97.18309859	,20},
	{0.972816901408451,	98.87323944	,20},
	{1.00605633802817,	100.5633803	,20},
	{1.03985915492958,	102.2535211	,20},
	{1.07422535211268,	103.943662	,20},
	{1.10914084507042,	105.5492958	,20},
	{1.14457746478873,	107.0704225	,20},
	{1.18050704225352,	108.5070423	,20},
	{1.2169014084507,	109.8591549	,20},
	{1.2537323943662,	111.1267606	,20},
	{1.29097183098592,	112.3098592	,20},
	{1.32859154929577,	113.4084507	,20},
	{1.36656338028169,	114.4225352	,20},
	{1.40485915492958,	115.3521127	,20},
	{1.44345070422535,	116.1971831	,20},
	{1.48230985915493,	116.9577465	,20},
	{1.52140845070423,	117.6338028	,20},
	{1.56071830985916,	118.2253521	,20},
	{1.60021126760563,	118.7323944	,20},
	{1.63985915492958,	119.1549296	,20},
	{1.6796338028169,	119.4929577	,20},
	{1.71950704225352,	119.7464789	,20},
	{1.75945070422535,	119.915493	,20},
	{1.79943661971831,	120	,20},
	{1.83943661971831,	120	,20},
	{1.87943661971831,	120	,20},
	{1.91943661971831,	120	,20},
	{1.95943661971831,	120	,20},
	{1.99943661971831,	120	,20},
	{2.03943661971831,	120	,20},
	{2.07943661971831,	120	,20},
	{2.11943661971831,	120	,20},
	{2.15943661971831,	120	,20},
	{2.19943661971831,	120	,20},
	{2.23943661971831,	120	,20},
	{2.27943661971831,	120	,20},
	{2.31943661971831,	120	,20},
	{2.35940845070423,	119.8309859	,20},
	{2.39930985915493,	119.5774648	,20},
	{2.43911267605634,	119.2394366	,20},
	{2.47878873239437,	118.8169014	,20},
	{2.51830985915493,	118.3098592	,20},
	{2.55764788732394,	117.7183099	,20},
	{2.59677464788733,	117.0422535	,20},
	{2.63566197183099,	116.2816901	,20},
	{2.67428169014085,	115.4366197	,20},
	{2.71260563380282,	114.5070423	,20},
	{2.75060563380282,	113.4929577	,20},
	{2.78825352112676,	112.3943662	,20},
	{2.82552112676056,	111.2112676	,20},
	{2.86238028169014,	109.943662	,20},
	{2.89880281690141,	108.5915493	,20},
	{2.93476056338028,	107.1549296	,20},
	{2.97022535211268,	105.6338028	,20},
	{3.00516901408451,	104.028169	,20},
	{3.03956338028169,	102.3380282	,20},
	{3.07338028169014,	100.5633803	,20},
	{3.10661971830986,	98.87323944	,20},
	{3.13929577464789,	97.18309859	,20},
	{3.17140845070423,	95.49295775	,20},
	{3.20295774647887,	93.8028169	,20},
	{3.23394366197183,	92.11267606	,20},
	{3.2643661971831,	90.42253521	,20},
	{3.29422535211268,	88.73239437	,20},
	{3.32352112676056,	87.04225352	,20},
	{3.35225352112676,	85.35211268	,20},
	{3.38042253521127,	83.66197183	,20},
	{3.40802816901409,	81.97183099	,20},
	{3.43507042253521,	80.28169014	,20},
	{3.46154929577465,	78.5915493	,20},
	{3.4874647887324,	76.90140845	,20},
	{3.51281690140845,	75.21126761	,20},
	{3.53760563380282,	73.52112676	,20},
	{3.56183098591549,	71.83098592	,20},
	{3.58549295774648,	70.14084507	,20},
	{3.60859154929578,	68.45070423	,20},
	{3.63112676056338,	66.76056338	,20},
	{3.6530985915493,	65.07042254	,20},
	{3.67450704225352,	63.38028169	,20},
	{3.69535211267606,	61.69014085	,20},
	{3.7156338028169,	60	,20},
	{3.73535211267606,	58.30985915	,20},
	{3.75450704225352,	56.61971831	,20},
	{3.7730985915493,	54.92957746	,20},
	{3.79112676056338,	53.23943662	,20},
	{3.80859154929578,	51.54929577	,20},
	{3.82549295774648,	49.85915493	,20},
	{3.84183098591549,	48.16901408	,20},
	{3.85760563380282,	46.47887324	,20},
	{3.87281690140845,	44.78873239	,20},
	{3.88746478873239,	43.09859155	,20},
	{3.90154929577465,	41.4084507	,20},
	{3.91507042253521,	39.71830986	,20},
	{3.92802816901408,	38.02816901	,20},
	{3.94042253521127,	36.33802817	,20},
	{3.95225352112676,	34.64788732	,20},
	{3.96352112676056,	32.95774648	,20},
	{3.97422535211268,	31.26760563	,20},
	{3.9843661971831,	29.57746479	,20},
	{3.99394366197183,	27.88732394	,20},
	{4.00295774647887,	26.1971831	,20},
	{4.01140845070423,	24.50704225	,20},
	{4.01929577464789,	22.81690141	,20},
	{4.02661971830986,	21.12676056	,20},
	{4.03338028169014,	19.43661972	,20},
	{4.03957746478873,	17.74647887	,20},
	{4.04521126760564,	16.05633803	,20},
	{4.05029577464789,	14.45070423	,20},
	{4.05485915492958,	12.92957746	,20},
	{4.05892957746479,	11.49295775	,20},
	{4.06253521126761,	10.14084507	,20},
	{4.06570422535211,	8.873239437	,20},
	{4.06846478873239,	7.690140845	,20},
	{4.07084507042254,	6.591549296	,20},
	{4.07287323943662,	5.577464789	,20},
	{4.07457746478873,	4.647887324	,20},
	{4.07598591549296,	3.802816901	,20},
	{4.07712676056338,	3.042253521	,20},
	{4.07802816901409,	2.366197183	,20},
	{4.07871830985916,	1.774647887	,20},
	{4.07922535211268,	1.267605634	,20},
	{4.07957746478873,	0.845070423	,20},
	{4.07980281690141,	0.507042254	,20},
	{4.07992957746479,	0.253521127	,20},
	{4.07998591549296,	0.084507042	,20},
	{4.08,	0	,20}};




}