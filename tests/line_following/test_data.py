import numpy as np

STRAIGHT = np.array([[391.0, 892.0], [391.0, 889.0440677966102], [391.0, 886.0881355932204], [391.0542372881356, 883.1322033898305], [391.2389830508474, 880.1762711864407], [391.4237288135593, 877.2203389830509], [391.5, 874.264406779661], [391.5, 871.3084745762712], [391.5, 868.3525423728813], [391.6627118644068, 865.3966101694915], [391.8474576271186, 862.4406779661017], [392.0, 859.4847457627119], [392.0, 856.5288135593221], [392.0, 853.5728813559322], [392.0864406779661, 850.6169491525424], [392.271186440678, 847.6610169491526], [392.4559322033898, 844.7050847457627], [392.5, 841.7491525423729], [392.5, 838.793220338983], [392.5101694915254, 835.8372881355932], [392.6949152542373, 832.8813559322034], [392.87966101694917, 829.9254237288136], [393.064406779661, 826.9694915254238], [393.2491525423729, 824.0135593220339], [393.43389830508477, 821.0576271186441], [393.5, 818.1016949152543], [393.5, 815.1457627118644], [393.5, 812.1898305084745], [393.6728813559322, 809.2338983050847], [393.8576271186441, 806.2779661016949], [394.04237288135596, 803.3220338983051], [394.2271186440678, 800.3661016949153], [394.41186440677967, 797.4101694915254], [394.59661016949156, 794.4542372881356], [394.7813559322034, 791.4983050847458], [394.96610169491527, 788.542372881356], [395.0, 785.586440677966], [395.0, 782.6305084745762], [395.02033898305086, 779.6745762711864], [395.20508474576275, 776.7186440677966], [395.3898305084746, 773.7627118644068], [395.5, 770.8067796610169], [395.5, 767.8508474576271], [395.5, 764.8949152542373], [395.628813559322, 761.9389830508475], [395.8135593220339, 758.9830508474577], [395.99830508474577, 756.0271186440677], [396.1830508474576, 753.0711864406779], [396.3677966101695, 750.1152542372881], [396.5, 747.1593220338983], [396.5, 744.2033898305085], [396.5, 741.2474576271186], [396.60677966101696, 738.2915254237288], [396.7915254237288, 735.335593220339], [396.97627118644067, 732.3796610169492], [397.0, 729.4237288135594], [397.0, 726.4677966101694], [397.0, 723.5118644067796], [397.0, 720.5559322033898], [397.0, 717.6], [397.08474576271186, 714.6440677966102], [397.26949152542375, 711.6881355932203], [397.4542372881356, 708.7322033898305], [397.63898305084746, 705.7762711864407], [397.82372881355934, 702.8203389830509], [398.00847457627117, 699.8644067796611], [398.19322033898305, 696.9084745762711], [398.37796610169494, 693.9525423728813], [398.5, 690.9966101694915], [398.5, 688.0406779661017], [398.5, 685.0847457627119], [398.61694915254236, 682.128813559322], [398.80169491525425, 679.1728813559322], [398.98644067796613, 676.2169491525424], [399.0, 673.2610169491526], [399.0, 670.3050847457628], [399.0406779661017, 667.3491525423728], [399.22542372881355, 664.393220338983], [399.41016949152544, 661.4372881355932], [399.5949152542373, 658.4813559322034], [399.77966101694915, 655.5254237288136], [399.96440677966103, 652.5694915254237], [400.0, 649.6135593220339], [400.0, 646.6576271186441], [400.01864406779663, 643.7016949152542], [400.20338983050846, 640.7457627118645], [400.38813559322034, 637.7898305084746], [400.57288135593217, 634.8338983050847], [400.75762711864405, 631.8779661016949], [400.94237288135594, 628.922033898305], [401.12711864406776, 625.9661016949153], [401.31186440677965, 623.0101694915254], [401.49661016949153, 620.0542372881356], [401.5, 617.0983050847458], [401.5, 614.1423728813559], [401.5508474576271, 611.1864406779662], [401.73559322033896, 608.2305084745763], [401.92033898305084, 605.2745762711864], [402.1050847457627, 602.3186440677966], [402.28983050847455, 599.3627118644067], [402.47457627118644, 596.406779661017], [402.5, 593.4508474576271], [402.5, 590.4949152542373], [402.52881355932203, 587.5389830508475], [402.7135593220339, 584.5830508474576], [402.89830508474574, 581.6271186440679], [403.0830508474576, 578.671186440678], [403.2677966101695, 575.7152542372881], [403.45254237288134, 572.7593220338983], [403.6372881355932, 569.8033898305084], [403.8220338983051, 566.8474576271186], [404.0, 563.8915254237288], [404.0, 560.935593220339], [404.0, 557.9796610169492], [404.06101694915253, 555.0237288135593], [404.2457627118644, 552.0677966101695], [404.4305084745763, 549.1118644067797], [404.5, 546.1559322033899], [404.5, 543.2], [404.5, 540.2440677966101], [404.6694915254237, 537.2881355932203], [404.8542372881356, 534.3322033898305], [405.0389830508475, 531.3762711864406], [405.2237288135593, 528.4203389830509], [405.4084745762712, 525.464406779661], [405.5, 522.5084745762712], [405.5, 519.5525423728814], [405.5, 516.5966101694914], [405.79491525423725, 513.6406779661017], [406.164406779661, 510.6847457627119], [406.51694915254234, 507.728813559322], [406.7016949152542, 504.7728813559322], [406.8864406779661, 501.81694915254235], [407.07118644067793, 498.86101694915254], [407.2559322033898, 495.90508474576274], [407.4406779661017, 492.9491525423729], [407.5, 489.99322033898306], [407.5, 487.0372881355932], [407.5, 484.0813559322034], [407.6796610169491, 481.12542372881353], [407.864406779661, 478.1694915254237], [408.0, 475.2135593220339], [408.0, 472.25762711864405], [408.0, 469.30169491525425], [408.0, 466.3457627118644], [408.0, 463.3898305084746], [408.0, 460.43389830508477], [408.3152542372881, 457.4779661016949], [408.6847457627119, 454.5220338983051], [409.0, 451.56610169491523], [409.0, 448.6101694915254], [409.0, 445.65423728813556], [409.0813559322034, 442.69830508474575], [409.2661016949153, 439.74237288135595], [409.4508474576271, 436.7864406779661], [409.5, 433.8305084745763], [409.5, 430.8745762711864], [409.5050847457627, 427.9186440677966], [409.6898305084746, 424.9627118644068], [409.87457627118647, 422.00677966101694], [410.0593220338983, 419.0508474576271], [410.2440677966102, 416.09491525423726], [410.42881355932207, 413.13898305084746], [410.5, 410.18305084745765], [410.5, 407.2271186440678], [410.5, 404.271186440678], [410.835593220339, 401.3152542372881], [411.2050847457627, 398.3593220338983], [411.52519468621165, 395.40338983050844], [411.6500229042602, 392.44745762711864], [411.77485112230875, 389.49152542372883], [411.8996793403573, 386.53559322033897], [412.02450755840584, 383.57966101694916], [412.14933577645445, 380.6237288135593], [412.27416399450294, 377.6677966101695], [412.39899221255155, 374.7118644067797], [412.5238204306001, 371.7559322033899], [412.64864864864865, 368.79999999999995], [412.7734768666972, 365.84406779661015], [412.89830508474574, 362.88813559322034], [413.02313330279435, 359.93220338983053], [413.14796152084284, 356.9762711864407], [413.27278973889145, 354.0203389830508], [413.39761795694, 351.064406779661], [413.52244617498854, 348.1084745762712], [413.6472743930371, 345.1525423728814], [413.77210261108564, 342.19661016949146], [413.89693082913425, 339.24067796610166], [414.02175904718274, 336.28474576271185], [414.14658726523135, 333.32881355932204], [414.27141548327984, 330.37288135593224], [414.39624370132844, 327.4169491525423], [414.521071919377, 324.4610169491525], [414.64590013742554, 321.5050847457627], [414.77072835547415, 318.5491525423729], [414.89555657352264, 315.5932203389831], [415.02038479157125, 312.63728813559317], [415.14521300961974, 309.68135593220336], [415.27004122766834, 306.72542372881355], [415.3948694457169, 303.76949152542375], [415.51969766376544, 300.81355932203394], [415.64452588181405, 297.857627118644], [415.76935409986254, 294.9016949152542], [415.89418231791115, 291.9457627118644], [416.01901053595964, 288.9898305084746], [416.14383875400824, 286.0338983050848], [416.26866697205685, 283.07796610169487], [416.39349519010534, 280.12203389830506], [416.51832340815395, 277.16610169491526], [416.6431516262025, 274.21016949152545], [416.76797984425104, 271.25423728813564], [416.8928080622996, 268.2983050847457], [417.01763628034814, 265.3423728813559], [417.14246449839675, 262.3864406779661], [417.26729271644524, 259.4305084745763], [417.39212093449385, 256.4745762711864], [417.5169491525424, 253.51864406779657], [417.64177737059094, 250.56271186440677], [417.7666055886395, 247.60677966101696], [417.89143380668804, 244.65084745762715], [418.01626202473665, 241.69491525423723], [418.14109024278514, 238.73898305084742], [418.26591846083375, 235.78305084745762], [418.39074667888224, 232.8271186440678], [418.51557489693084, 229.871186440678], [418.6404031149794, 226.91525423728808], [418.76523133302794, 223.95932203389827], [418.89005955107655, 221.00338983050847], [419.01488776912504, 218.04745762711866], [419.13971598717364, 215.09152542372885], [419.26454420522214, 212.13559322033893], [419.38937242327074, 209.17966101694913], [419.5142006413193, 206.22372881355932], [419.63902885936784, 203.2677966101695], [419.7638570774164, 200.3118644067797], [419.88868529546494, 197.35593220338978], [420.01351351351354, 194.39999999999998], [420.13834173156204, 191.44406779661017], [420.26316994961064, 188.48813559322036], [420.3879981676592, 185.53220338983056], [420.51282638570774, 182.57627118644064], [420.6376546037563, 179.62033898305083], [420.76248282180484, 176.66440677966102], [420.88731103985344, 173.70847457627121], [421.01213925790194, 170.7525423728813], [421.13696747595054, 167.7966101694915], [421.2617956939991, 164.84067796610168], [421.38662391204764, 161.88474576271187], [421.5114521300962, 158.92881355932207], [421.63628034814474, 155.97288135593215], [421.76110856619334, 153.01694915254234], [421.88593678424184, 150.06101694915253], [422.01076500229044, 147.10508474576272], [422.135593220339, 144.14915254237292], [422.26042143838754, 141.193220338983], [422.3852496564361, 138.2372881355932], [422.51007787448464, 135.28135593220338], [422.63490609253324, 132.32542372881358], [422.75973431058173, 129.36949152542377], [422.88456252863034, 126.41355932203385], [423.0093907466789, 123.45762711864404], [423.13421896472744, 120.50169491525423], [423.259047182776, 117.54576271186443], [423.38387540082454, 114.58983050847462], [423.50870361887314, 111.6338983050847], [423.63353183692163, 108.67796610169489], [423.75836005497024, 105.72203389830509], [423.8831882730188, 102.76610169491528], [424.0, 99.81016949152547], [424.0, 96.85423728813555], [424.0, 93.89830508474574], [424.06610169491523, 90.94237288135594], [424.2508474576271, 87.98644067796613], [424.435593220339, 85.03050847457621], [424.5, 82.0745762711864], [424.5, 79.1186440677966], [424.5, 76.16271186440679], [424.5, 73.20677966101698], [424.5, 70.25084745762706], [424.5, 67.29491525423725], [424.5, 64.33898305084745], [424.5, 61.38305084745764], [424.5983050847458, 58.42711864406783], [424.7830508474576, 55.47118644067791], [424.9677966101695, 52.515254237288104], [425.0, 49.5593220338983], [425.0, 46.60338983050849], [425.02203389830504, 43.64745762711868], [425.2067796610169, 40.69152542372876], [425.3915254237288, 37.735593220338956], [425.6525423728814, 34.77966101694915], [426.02203389830504, 31.82372881355934], [426.3915254237288, 28.867796610169535], [426.5, 25.911864406779614], [426.5, 22.955932203389807], [426.5, 20.0]])
CORNER = np.array([[402.0, 892.0], [401.66666666666663, 887.1111111111111], [400.33333333333337, 882.2222222222222], [399.5, 877.3333333333333], [398.1666666666667, 872.4444444444445], [396.77777777777777, 867.5555555555555], [395.33333333333337, 862.6666666666667], [393.3888888888889, 857.7777777777778], [391.44444444444446, 852.8888888888889], [389.0, 848.0], [385.94444444444446, 843.1111111111111], [382.3888888888889, 838.2222222222222], [378.8333333333333, 833.3333333333333], [375.27777777777777, 828.4444444444445], [371.1111111111111, 823.5555555555555], [366.83333333333337, 818.6666666666667], [362.05555555555554, 813.7777777777778], [356.7777777777778, 808.8888888888889], [351.5, 804.0], [345.1111111111111, 799.1111111111111], [338.22222222222223, 794.2222222222222], [331.3333333333333, 789.3333333333333], [322.94444444444446, 784.4444444444445], [314.88888888888886, 779.5555555555555], [304.6666666666667, 774.6666666666667], [293.94444444444446, 769.7777777777778], [282.2222222222223, 764.8888888888889], [269.0, 760.0]])
CROSSING_LANE_1 = np.array([[218.5, 704.0], [217.4754601226994, 700.0736196319019], [216.57668711656441, 696.1472392638036], [215.89877300613497, 692.2208588957055], [215.11349693251535, 688.2944785276073], [214.33128834355827, 684.3680981595091], [213.9325153374233, 680.4417177914111], [213.5644171779141, 676.5153374233129], [213.07461834783848, 672.5889570552147], [212.5844628334998, 668.6625766871166], [212.01762020259665, 664.7361963190184], [211.45077757169355, 660.8098159509202], [211.15387359109715, 656.8834355828221], [210.94285918105294, 652.9570552147239], [210.7318447710087, 649.0306748466257], [210.35211870452275, 645.1042944785275], [209.96319018404907, 641.1779141104295], [209.5497217862748, 637.2515337423313], [208.98287915537168, 633.3251533742332], [208.41603652446855, 629.398773006135], [207.84919389356543, 625.4723926380368], [207.2823512626623, 621.5460122699387], [206.71550863175918, 617.6196319018404], [206.24069054073334, 613.6932515337423], [205.85176202025966, 609.7668711656441], [205.462833499786, 605.840490797546], [205.07390497931232, 601.9141104294479], [204.6849764588386, 597.9877300613497], [204.29604793836495, 594.0613496932515], [203.7476102154373, 590.1349693251534], [203.18076758453418, 586.2085889570552], [202.62926237694393, 582.282208588957], [202.24033385647027, 578.3558282208589], [201.85140533599656, 574.4294478527607], [201.51155657012413, 570.5030674846626], [201.30054216007989, 566.5766871165644], [201.08952775003567, 562.6503067484663], [200.71286916821228, 558.7239263803681], [200.14602653730918, 554.7975460122699], [199.57918390640603, 550.8711656441718], [199.01234127550293, 546.9447852760736], [198.4454986445998, 543.0184049079755], [197.87865601369668, 539.0920245398772], [197.61242687972606, 535.1656441717791], [197.40141246968184, 531.239263803681], [197.1781281209873, 527.3128834355828], [196.61128549008419, 523.3865030674847], [196.04444285918106, 519.4601226993865], [195.47760022827794, 515.5337423312883], [194.9107575973748, 511.60736196319016], [194.3439149664717, 507.680981595092], [193.92431159937223, 503.7546012269938], [193.71329718932802, 499.8282208588957], [193.50228277928377, 495.9018404907975], [192.1102867741475, 491.97546012269936], [189.94221714937933, 488.0490797546012], [187.77414752461124, 484.1226993865031], [185.88828648880013, 480.1963190184049], [184.07604508489084, 476.2699386503067], [182.2638036809816, 472.3435582822086], [182.37487516050794, 468.4171779141104], [182.51968897132258, 464.49079754601223], [182.9405764017691, 460.5644171779141], [184.68661720644883, 456.63803680981596], [186.43265801112852, 452.7116564417178], [187.59894421458125, 448.7852760736196], [187.7437580253959, 444.85889570552143], [187.8885718362106, 440.9325153374233], [187.44442859181052, 437.00613496932516], [186.5217577400485, 433.079754601227], [185.59908688828648, 429.15337423312883], [185.33592523897846, 425.22699386503064], [185.30282493936366, 421.3006134969325], [185.26972463974892, 417.37423312883436], [184.4084034812384, 413.44785276073617], [183.4857326294764, 409.521472392638], [182.67042374090454, 405.59509202453984], [182.51533742331287, 401.6687116564417], [182.0674846625767, 397.7423312883435], [181.56134969325154, 393.8159509202454], [180.5368098159509, 389.88957055214723], [179.51226993865032, 385.96319018404904], [179.11042944785277, 382.0368098159509], [179.3312883435583, 378.11042944785277], [179.55214723926383, 374.1840490797546], [178.91411042944785, 370.25766871165644], [177.88957055214723, 366.3312883435583], [176.86503067484662, 362.4049079754601], [176.64519427402863, 358.47852760736197], [176.71881390593046, 354.55214723926383], [176.7433537832311, 350.62576687116564], [176.10531697341514, 346.69938650306744], [175.46728016359918, 342.7730061349693], [174.92126789366054, 338.84662576687117], [174.6390593047035, 334.920245398773], [174.35685071574642, 330.9938650306748], [174.15439672801637, 327.06748466257665], [174.05010224948876, 323.1411042944785], [173.83742331288343, 319.2147239263803], [173.07055214723925, 315.28834355828224], [172.04601226993867, 311.36196319018404], [171.02147239263803, 307.43558282208585], [171.0276073619632, 303.5092024539877], [171.4754601226994, 299.5828220858896], [171.9079754601227, 295.6564417177914], [170.57055214723925, 291.7300613496932], [169.73006134969324, 287.8036809815951], [169.44171779141104, 283.8773006134969], [169.1233810497614, 279.9509202453987], [168.7716428084526, 276.0245398773005], [168.56100886162235, 272.09815950920245], [168.56509884117247, 268.1717791411043], [168.56918882072256, 264.2453987730061], [168.05180640763464, 260.319018404908], [167.1663258350375, 256.3926380368098], [166.28084526244035, 252.46625766871165], [165.947511929107, 248.53987730061345], [165.77368779822768, 244.61349693251532], [165.59986366734833, 240.68711656441712], [165.42603953646898, 236.760736196319], [165.25221540558965, 232.83435582822085], [164.96796182685753, 228.90797546012266], [163.83435582822085, 224.98159509202452], [162.4969325153374, 221.05521472392635], [161.70245398773005, 217.12883435582822], [161.5, 213.20245398773002], [161.5, 209.2760736196319], [161.2638036809816, 205.3496932515337], [160.95092024539878, 201.42331288343556], [160.63803680981596, 197.49693251533742], [159.98466257668713, 193.57055214723923], [159.2730061349693, 189.6441717791411], [157.9601226993865, 185.7177914110429], [157.14647239263803, 181.79141104294476], [156.71242331288343, 177.8650306748466], [156.37039877300614, 173.93865030674846], [156.82592024539878, 170.01226993865026], [157.2814417177914, 166.08588957055213], [157.37193251533742, 162.159509202454], [156.58205521472394, 158.2331288343558], [155.79217791411043, 154.30674846625766], [155.00230061349694, 150.38036809815947], [154.21242331288343, 146.45398773006133], [153.42254601226992, 142.52760736196313], [153.70935582822085, 138.601226993865], [154.5207055214724, 134.6748466257668], [155.1963190184049, 130.7484662576687], [154.54601226993867, 126.82208588957056], [153.3926380368098, 122.89570552147237], [152.34355828220862, 118.96932515337423], [151.31901840490798, 115.04294478527603], [150.5398773006135, 111.1165644171779], [149.85582822085888, 107.1901840490797], [149.18711656441718, 103.26380368098157], [148.51840490797545, 99.33742331288337], [148.30981595092027, 95.41104294478524], [148.7085889570552, 91.4846625766871], [149.23006134969324, 87.5582822085889], [149.05828220858896, 83.6319018404908], [148.09815950920245, 79.7055214723926], [147.0122699386503, 75.77914110429447], [145.81901840490798, 71.85276073619627], [144.84662576687117, 67.92638036809814], [144.0, 64.0]])
CROSSING_LANE_0 = np.array([[405.5, 876.0], [405.52147239263803, 870.8220858895706], [405.3312883435583, 865.6441717791411], [405.00306748466255, 860.4662576687117], [405.0858895705521, 855.2883435582821], [404.8282208588957, 850.1104294478528], [404.50613496932516, 844.9325153374233], [404.6503067484663, 839.7546012269938], [404.49793123127404, 834.5766871165645], [404.31452418319304, 829.3987730061349], [405.1065772578114, 824.2208588957055], [405.7207162220003, 819.0429447852761], [406.14160365244686, 813.8650306748466], [405.59623341418177, 808.6871165644172], [404.86988158082465, 803.5092024539878], [403.9594806677129, 798.3312883435583], [402.41104294478527, 793.1533742331288], [401.41475246112145, 787.9754601226994], [400.8877871308318, 782.79754601227], [400.67677272078754, 777.6196319018405], [400.4256619040825, 772.4417177914111], [400.1429998165624, 767.2638036809816], [399.86033772904227, 762.0858895705521], [399.57767564152215, 756.9079754601228], [399.295013554002, 751.7300613496932], [399.01235146648185, 746.5521472392638], [398.7296893789618, 741.3742331288342], [398.4470272914416, 736.1963190184049], [398.1643652039215, 731.0184049079755], [397.88170311640135, 725.840490797546], [397.59904102888123, 720.6625766871166], [397.3163789413611, 715.4846625766871], [397.033716853841, 710.3067484662577], [396.75105476632086, 705.1288343558282], [396.46839267880074, 699.9509202453987], [396.1857305912806, 694.7730061349694], [395.90306850376044, 689.5950920245399], [395.62040641624037, 684.4171779141104], [395.3377443287202, 679.239263803681], [395.05508224120007, 674.0613496932515], [394.77242015367995, 668.8834355828221], [394.4897580661598, 663.7055214723927], [394.2070959786397, 658.5276073619632], [393.9244338911196, 653.3496932515337], [393.64177180359945, 648.1717791411043], [393.35910971607933, 642.9938650306749], [393.0764476285592, 637.8159509202454], [392.79378554103903, 632.638036809816], [392.51112345351896, 627.4601226993865], [392.2284613659988, 622.282208588957], [391.9457992784787, 617.1042944785277], [391.66313719095854, 611.9263803680981], [391.38047510343847, 606.7484662576687], [391.0978130159183, 601.5705521472393], [390.8151509283982, 596.3926380368098], [390.53248884087805, 591.2147239263804], [390.249826753358, 586.0368098159508], [389.9671646658378, 580.8588957055215], [389.6845025783176, 575.680981595092], [389.40184049079755, 570.5030674846626], [389.11917840327743, 565.3251533742332], [388.8365163157573, 560.1472392638036], [388.60315308888573, 554.9693251533743], [388.3921386788415, 549.7914110429448], [388.18112426879725, 544.6134969325153], [387.7369810243972, 539.435582822086], [387.25909544870876, 534.2576687116564], [387.0480810386646, 529.079754601227], [386.83706662862033, 523.9018404907976], [386.6260522185761, 518.7239263803681], [386.4150378085319, 513.5460122699387], [385.96475959480665, 508.36809815950915], [385.4930089884434, 503.19018404907973], [385.28199457839924, 498.0122699386503], [385.07098016835494, 492.83435582822085], [384.85996575831075, 487.6564417177914], [384.6489513482665, 482.4785276073619], [384.07055214723925, 477.30061349693244], [383.1901840490798, 472.1226993865031], [382.782208588957, 466.9447852760736], [382.717791411043, 461.76687116564415], [382.49079754601223, 456.5889570552147], [381.50920245398777, 451.41104294478527], [381.03067484662574, 446.2331288343558], [380.717791411043, 441.0552147239264], [380.4049079754601, 435.8773006134969], [380.0920245398773, 430.69938650306744], [379.77914110429447, 425.52147239263803], [379.4887525562372, 420.34355828220856], [379.1267893660532, 415.16564417177915], [378.8721881390593, 409.9877300613497], [379.1022494887526, 404.8098159509202], [378.8108384458078, 399.63190184049074], [378.4672801635992, 394.4539877300613], [378.3629856850716, 389.27607361963186], [378.258691206544, 384.09815950920245], [378.0562372188139, 378.920245398773], [377.61758691206546, 373.74233128834356], [377.6042944785276, 368.56441717791404], [377.4233128834356, 363.3865030674846], [376.7760736196319, 358.2085889570552], [376.398773006135, 353.03067484662574], [375.9815950920246, 347.8527607361963], [375.56134969325154, 342.6748466257668], [375.69325153374234, 337.4969325153374], [375.1871165644171, 332.3190184049079], [374.7239263803681, 327.1411042944785], [375.2546012269939, 321.96319018404904], [375.11724608043625, 316.7852760736196], [374.27471029311516, 311.6073619631901], [374.26959781867754, 306.4294478527607], [374.20313565098843, 301.2515337423313], [373.6949556918882, 296.0736196319018], [373.4689843217451, 290.8957055214724], [373.2951601908657, 285.7177914110429], [373.12133605998633, 280.53987730061345], [372.6622358554874, 275.361963190184], [372.39331970006816, 270.1840490797546], [372.5538513974097, 265.0061349693251], [371.5610088616223, 259.8282208588957], [370.87491479209274, 254.65030674846622], [371.0354464894342, 249.47239263803675], [370.6564417177914, 244.29447852760734], [369.9049079754601, 239.11656441717787], [369.24233128834356, 233.93865030674846], [369.29447852760734, 228.760736196319], [369.3711656441718, 223.58282208588952], [368.8006134969325, 218.40490797546005], [368.4509202453988, 213.22699386503064], [368.26993865030676, 208.04907975460122], [368.46625766871165, 202.87116564417175], [368.19938650306744, 197.69325153374228], [367.2638036809816, 192.51533742331281], [366.5023006134969, 187.3374233128834], [366.69708588957053, 182.15950920245393], [366.8151840490798, 176.98159509202452], [366.73696319018404, 171.80368098159505], [366.045245398773, 166.62576687116558], [365.72162576687117, 161.44785276073617], [365.97776073619633, 156.2699386503067], [365.9240797546012, 151.0920245398773], [365.5575153374233, 145.91411042944782], [364.81058282208585, 140.73619631901835], [364.6894171779141, 135.55828220858888], [364.7584355828221, 130.38036809815947], [365.01457055214723, 125.20245398773], [364.819018404908, 120.02453987730058], [364.20552147239266, 114.84662576687117], [362.9171779141104, 109.66871165644164], [362.3803680981595, 104.49079754601223], [362.22085889570553, 99.31288343558276], [362.4877300613497, 94.13496932515335], [362.44171779141107, 88.95705521472388], [361.8159509202454, 83.77914110429447], [360.83435582822085, 78.60122699386494], [360.50306748466255, 73.42331288343553], [360.3496932515337, 68.24539877300612], [360.49386503067484, 63.067484662576646], [360.5, 57.889570552147234], [359.7638036809816, 52.71165644171771], [358.3865030674846, 47.533742331288295], [358.0828220858896, 42.355828220858825], [357.8128834355828, 37.17791411042941], [357.5, 32.0]])
STOPLINE = np.array([[391.5, 892.0], [391.3152542372882, 889.0440677966102], [391.1305084745763, 886.0881355932204], [391.0542372881356, 883.1322033898305], [391.2389830508474, 880.1762711864407], [391.4237288135593, 877.2203389830509], [391.5, 874.264406779661], [391.5, 871.3084745762712], [391.5, 868.3525423728813], [392.80169491525425, 865.3966101694915], [394.27966101694915, 862.4406779661017], [395.33898305084745, 859.4847457627119], [394.41525423728814, 856.5288135593221], [393.49152542372883, 853.5728813559322], [392.9135593220339, 850.6169491525424], [392.728813559322, 847.6610169491526], [392.5440677966102, 844.7050847457627], [392.5, 841.7491525423729], [392.5, 838.793220338983], [392.5101694915254, 835.8372881355932], [392.6949152542373, 832.8813559322034], [392.87966101694917, 829.9254237288136], [393.064406779661, 826.9694915254238], [393.2491525423729, 824.0135593220339], [393.43389830508477, 821.0576271186441], [393.6186440677966, 818.1016949152543], [393.8033898305085, 815.1457627118644], [393.98813559322036, 812.1898305084745], [394.0, 809.2338983050847], [394.0, 806.2779661016949], [394.04237288135596, 803.3220338983051], [394.2271186440678, 800.3661016949153], [394.41186440677967, 797.4101694915254], [394.59661016949156, 794.4542372881356], [394.7813559322034, 791.4983050847458], [394.96610169491527, 788.542372881356], [395.0, 785.586440677966], [395.0, 782.6305084745762], [395.02033898305086, 779.6745762711864], [395.20508474576275, 776.7186440677966], [395.3898305084746, 773.7627118644068], [395.5, 770.8067796610169], [395.5, 767.8508474576271], [395.5, 764.8949152542373], [395.5, 761.9389830508475], [395.5, 758.9830508474577], [395.5, 756.0271186440677], [395.6830508474576, 753.0711864406779], [395.8677966101695, 750.1152542372881], [396.05254237288136, 747.1593220338983], [396.2372881355932, 744.2033898305085], [396.4220338983051, 741.2474576271186], [396.60677966101696, 738.2915254237288], [396.7915254237288, 735.335593220339], [396.97627118644067, 732.3796610169492], [397.0, 729.4237288135594], [397.0, 726.4677966101694], [397.03050847457627, 723.5118644067796], [397.21525423728815, 720.5559322033898], [397.4, 717.6], [397.5, 714.6440677966102], [397.5, 711.6881355932203], [397.5, 708.7322033898305], [397.63898305084746, 705.7762711864407], [397.82372881355934, 702.8203389830509], [398.00847457627117, 699.8644067796611], [398.19322033898305, 696.9084745762711], [398.37796610169494, 693.9525423728813], [398.5, 690.9966101694915], [398.5, 688.0406779661017], [398.5, 685.0847457627119], [398.61694915254236, 682.128813559322], [398.80169491525425, 679.1728813559322], [398.98644067796613, 676.2169491525424], [399.0, 673.2610169491526], [399.0, 670.3050847457628], [399.0, 667.3491525423728], [399.0, 664.393220338983], [399.0, 661.4372881355932], [399.0949152542373, 658.4813559322034], [399.27966101694915, 655.5254237288136], [399.46440677966103, 652.5694915254237], [399.7983050847458, 649.6135593220339], [400.1677966101695, 646.6576271186441], [400.5, 643.7016949152542], [400.5, 640.7457627118645], [400.5, 637.7898305084746], [400.5, 634.8338983050847], [400.5, 631.8779661016949], [400.5, 628.922033898305], [400.7542372881356, 625.9661016949153], [401.1237288135593, 623.0101694915254], [401.49322033898306, 620.0542372881356], [401.5, 617.0983050847458], [401.5, 614.1423728813559], [401.5508474576271, 611.1864406779662], [401.73559322033896, 608.2305084745763], [401.92033898305084, 605.2745762711864], [402.0, 602.3186440677966], [402.0, 599.3627118644067], [402.0, 596.406779661017], [402.1593220338983, 593.4508474576271], [402.34406779661015, 590.4949152542373], [402.52881355932203, 587.5389830508475], [402.7135593220339, 584.5830508474576], [402.89830508474574, 581.6271186440679], [403.0, 578.671186440678], [403.0, 575.7152542372881], [403.0, 572.7593220338983], [403.27457627118645, 569.8033898305084], [403.64406779661016, 566.8474576271186], [404.0, 563.8915254237288], [404.0, 560.935593220339], [404.0, 557.9796610169492], [404.06101694915253, 555.0237288135593], [404.2457627118644, 552.0677966101695], [404.4305084745763, 549.1118644067797], [404.5, 546.1559322033899], [404.5, 543.2], [404.5, 540.2440677966101], [404.83898305084745, 537.2881355932203], [405.2084745762712, 534.3322033898305], [405.5, 531.3762711864406], [405.5, 528.4203389830509], [405.5, 525.464406779661], [405.5932203389831, 522.5084745762712], [405.7779661016949, 519.5525423728814], [405.9627118644068, 516.5966101694914], [406.1474576271186, 513.6406779661017], [406.3322033898305, 510.6847457627119], [406.5, 507.728813559322], [406.5, 504.7728813559322], [406.5, 501.81694915254235], [406.57118644067793, 498.86101694915254], [406.7559322033898, 495.90508474576274], [406.9406779661017, 492.9491525423729], [407.12542372881353, 489.99322033898306], [407.3101694915254, 487.0372881355932], [407.4949152542373, 484.0813559322034], [407.6796610169491, 481.12542372881353], [407.864406779661, 478.1694915254237], [408.0, 475.2135593220339], [408.0, 472.25762711864405], [408.0, 469.30169491525425], [408.1033898305085, 466.3457627118644], [408.2881355932203, 463.3898305084746], [408.4728813559322, 460.43389830508477], [408.6576271186441, 457.4779661016949], [408.8423728813559, 454.5220338983051], [409.0271186440678, 451.56610169491523], [409.2118644067797, 448.6101694915254], [409.3966101694915, 445.65423728813556], [409.5, 442.69830508474575], [409.5, 439.74237288135595], [409.5, 436.7864406779661], [409.635593220339, 433.8305084745763], [409.8203389830509, 430.8745762711864], [410.0050847457627, 427.9186440677966], [410.1898305084746, 424.9627118644068], [410.37457627118647, 422.00677966101694], [410.5, 419.0508474576271], [410.5, 416.09491525423726], [410.5, 413.13898305084746], [410.5, 410.18305084745765], [410.5, 407.2271186440678], [410.5, 404.271186440678], [410.835593220339, 401.3152542372881], [411.20508474576275, 398.3593220338983], [411.52418689876316, 395.40338983050844], [411.6440219880898, 392.44745762711864], [411.7638570774164, 389.49152542372883], [411.883692166743, 386.53559322033897], [412.0035272560696, 383.57966101694916], [412.12336234539623, 380.6237288135593], [412.24319743472284, 377.6677966101695], [412.36303252404946, 374.7118644067797], [412.48286761337613, 371.7559322033899], [412.6027027027027, 368.79999999999995], [412.7225377920293, 365.84406779661015], [412.8423728813559, 362.88813559322034], [412.9622079706826, 359.93220338983053], [413.08204306000914, 356.9762711864407], [413.20187814933576, 354.0203389830508], [413.32171323866237, 351.064406779661], [413.44154832798904, 348.1084745762712], [413.5613834173156, 345.1525423728814], [413.68121850664227, 342.19661016949146], [413.8010535959688, 339.24067796610166], [413.9208886852955, 336.28474576271185], [414.04072377462205, 333.32881355932204], [414.1605588639487, 330.37288135593224], [414.2803939532753, 327.4169491525423], [414.40022904260195, 324.4610169491525], [414.5200641319285, 321.5050847457627], [414.6398992212552, 318.5491525423729], [414.75973431058173, 315.5932203389831], [414.8795693999084, 312.63728813559317], [414.99940448923496, 309.68135593220336], [415.11923957856163, 306.72542372881355], [415.2390746678882, 303.76949152542375], [415.35890975721486, 300.81355932203394], [415.4787448465415, 297.857627118644], [415.5985799358681, 294.9016949152542], [415.71841502519464, 291.9457627118644], [415.8382501145213, 288.9898305084746], [415.95808520384793, 286.0338983050848], [416.07792029317454, 283.07796610169487], [416.1977553825011, 280.12203389830506], [416.31759047182777, 277.16610169491526], [416.4374255611544, 274.21016949152545], [416.557260650481, 271.25423728813564], [416.6770957398076, 268.2983050847457], [416.7969308291342, 265.3423728813559], [416.91676591846084, 262.3864406779661], [417.03660100778745, 259.4305084745763], [417.15643609711407, 256.4745762711864], [417.2762711864407, 253.51864406779657], [417.3961062757673, 250.56271186440677], [417.5159413650939, 247.60677966101696], [417.6357764544205, 244.65084745762715], [417.75561154374714, 241.69491525423723], [417.87544663307375, 238.73898305084742], [417.99528172240036, 235.78305084745762], [418.115116811727, 232.8271186440678], [418.2349519010536, 229.871186440678], [418.3547869903802, 226.91525423728808], [418.4746220797068, 223.95932203389827], [418.5944571690335, 221.00338983050847], [418.71429225836005, 218.04745762711866], [418.83412734768666, 215.09152542372885], [418.9539624370133, 212.13559322033893], [419.07379752633994, 209.17966101694913], [419.1936326156665, 206.22372881355932], [419.3134677049931, 203.2677966101695], [419.43330279431973, 200.3118644067797], [419.5531378836464, 197.35593220338978], [419.67297297297296, 194.39999999999998], [419.79280806229957, 191.44406779661017], [419.9126431516262, 188.48813559322036], [420.03247824095286, 185.53220338983056], [420.1523133302794, 182.57627118644064], [420.272148419606, 179.62033898305083], [420.39198350893264, 176.66440677966102], [420.5118185982593, 173.70847457627121], [420.63165368758587, 170.7525423728813], [420.75148877691254, 167.7966101694915], [420.8713238662391, 164.84067796610168], [420.99115895556577, 161.88474576271187], [421.1109940448923, 158.92881355932207], [421.230829134219, 155.97288135593215], [421.35066422354555, 153.01694915254234], [421.4704993128722, 150.06101694915253], [421.5903344021988, 147.10508474576272], [421.71016949152545, 144.14915254237292], [421.830004580852, 141.193220338983], [421.9498396701787, 138.2372881355932], [422.0696747595053, 135.28135593220338], [422.1895098488319, 132.32542372881358], [422.30934493815846, 129.36949152542377], [422.42918002748513, 126.41355932203385], [422.54901511681174, 123.45762711864404], [422.66885020613836, 120.50169491525423], [422.7886852954649, 117.54576271186443], [422.9085203847916, 114.58983050847462], [423.0283554741182, 111.6338983050847], [423.1481905634448, 108.67796610169489], [423.26802565277137, 105.72203389830509], [423.38786074209804, 102.76610169491528], [423.51186440677964, 99.81016949152547], [423.6966101694915, 96.85423728813555], [423.8813559322034, 93.89830508474574], [424.0, 90.94237288135594], [424.0, 87.98644067796613], [424.0, 85.03050847457621], [424.12033898305083, 82.0745762711864], [424.3050847457627, 79.1186440677966], [424.4898305084746, 76.16271186440679], [424.3254237288136, 73.20677966101698], [424.1406779661017, 70.25084745762706], [424.0440677966102, 67.29491525423725], [424.228813559322, 64.33898305084745], [424.4135593220339, 61.38305084745764], [424.5, 58.42711864406783], [424.5, 55.47118644067791], [424.5, 52.515254237288104], [424.6525423728814, 49.5593220338983], [424.8372881355932, 46.60338983050849], [425.02203389830504, 43.64745762711868], [425.206779661017, 40.69152542372876], [425.3915254237288, 37.735593220338956], [425.5762711864407, 34.77966101694915], [425.7610169491525, 31.82372881355934], [425.9457627118644, 28.867796610169535], [426.0, 25.911864406779614], [426.0, 22.955932203389807], [426.0, 20.0]])