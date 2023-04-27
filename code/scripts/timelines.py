# importing image object from PIL
import math
from PIL import Image, ImageDraw
  
# Data is in the format: (pid, tag, tstart, tend)
HEIGHT = 20
MARGIN = 0

# Agent 8:
NUMPROCS = 8
data = [
	[0,0,0.844000,2.645000], [0,0,19.393000,35.486000], [0,0,1824.844000,1826.692000], [0,0,3844.900000,3845.916000], [0,0,4867.706000,4884.248000], [0,0,5029.081000,5030.094000], [0,0,6727.968000,6729.055000], [0,0,6858.257000,6859.482000], [0,0,7492.391000,7507.359000], [0,0,8631.006000,8632.037000], [0,0,9156.888000,9158.442000], [0,0,11543.079000,11544.208000], [0,0,14021.090000,14036.042000], [0,0,15071.879000,15087.868000], [0,0,15223.476000,15238.846000], [0,0,15496.058000,15497.308000], [0,0,15885.616000,15886.848000], [0,0,16024.610000,16025.991000], [0,0,17888.366000,17889.518000], [0,0,19499.459000,19500.917000], [0,0,20133.706000,20134.799000], [0,0,21271.599000,21272.655000], [0,0,21632.813000,21633.855000], [0,1,21789.671000,25925.919000], [0,3,25927.792000,25928.730000], [0,0,26089.074000,26090.398000], [0,0,26202.373000,26209.193000], [0,0,26765.523000,26765.460000], [0,0,26817.919000,26819.079000], [0,0,26874.724000,26878.628000], [0,0,26896.196000,26897.269000], [0,0,26908.041000,26909.334000], [0,0,26916.194000,26923.268000], [0,0,27132.347000,27133.979000], [0,0,27255.430000,27261.971000], [0,0,27284.273000,27285.990000], [0,0,27409.646000,27410.750000], [0,0,27642.656000,27645.989000], [0,0,27806.431000,27817.203000], [0,1,27889.664000,27890.231000], [0,3,27892.511000,27892.866000], 
	[1,0,1824.841000,1825.961000], [1,0,2365.298000,2366.548000], [1,0,2876.812000,2878.055000], [1,0,3956.038000,3959.252000], [1,0,5569.632000,5586.626000], [1,0,7167.464000,7184.287000], [1,0,7925.604000,7942.430000], [1,0,7953.421000,7955.059000], [1,0,8241.891000,8243.389000], [1,0,8488.067000,8489.210000], [1,0,9918.953000,9935.956000], [1,0,13907.985000,13909.060000], [1,0,14579.112000,14580.552000], [1,0,14904.613000,14905.843000], [1,0,15223.464000,15240.261000], [1,0,15449.253000,15450.509000], [1,0,15495.111000,15496.499000], [1,0,16904.936000,16906.074000], [1,0,19375.012000,19376.193000], [1,0,19520.772000,19521.880000], [1,1,25721.299000,25925.919000], [1,3,25927.884000,25928.730000], [1,0,26201.842000,26208.666000], [1,0,26208.905000,26210.172000], [1,0,26775.531000,26786.898000], [1,0,26875.030000,26880.843000], [1,0,26916.613000,27020.637000], [1,0,27037.358000,27038.785000], [1,0,27254.156000,27262.381000], [1,0,27687.016000,27688.262000], [1,0,27806.242000,27816.837000], [1,0,27817.160000,27818.921000], [1,1,27888.284000,27890.231000], [1,3,27892.606000,27892.866000], 
	[2,0,0.872000,2.028000], [2,0,35.797000,35.755000], [2,0,265.648000,266.689000], [2,0,893.294000,894.304000], [2,0,2875.746000,2877.106000], [2,0,3499.622000,3500.738000], [2,0,3621.864000,3622.866000], [2,0,4646.122000,4647.173000], [2,0,6425.791000,6427.164000], [2,0,6427.959000,6444.974000], [2,0,10745.231000,10746.587000], [2,0,10870.224000,10871.431000], [2,0,12542.563000,12543.696000], [2,0,14020.848000,14037.500000], [2,0,14409.715000,14410.910000], [2,0,14905.121000,14906.273000], [2,0,15047.788000,15064.217000], [2,0,15655.609000,15656.644000], [2,0,15890.942000,15892.020000], [2,0,16013.892000,16015.264000], [2,0,18336.865000,18337.965000], [2,0,20834.769000,20836.046000], [2,0,21150.956000,21152.734000], [2,0,21264.769000,21266.285000], [2,1,21781.301000,25925.919000], [2,3,25928.487000,25928.730000], [2,0,25930.214000,25931.686000], [2,0,26737.197000,26738.517000], [2,0,26775.877000,26786.516000], [2,0,26786.832000,26788.386000], [2,0,26873.907000,26880.068000], [2,0,26915.620000,27021.510000], [2,0,27249.851000,27251.134000], [2,0,27266.574000,27283.760000], [2,0,27486.993000,27486.853000], [2,0,27738.607000,27739.651000], [2,0,27806.565000,27818.090000], [2,0,27818.486000,27819.634000], [2,1,27887.748000,27890.231000], [2,3,27892.584000,27892.866000], 
	[3,0,0.838000,3.575000], [3,0,12.862000,13.942000], [3,0,37.824000,38.927000], [3,0,3138.248000,3139.262000], [3,0,5766.465000,5767.734000], [3,0,9616.992000,9618.005000], [3,0,10260.520000,10276.970000], [3,0,13475.964000,13492.354000], [3,0,15072.657000,15088.874000], [3,0,15197.745000,15198.997000], [3,0,16866.752000,16867.802000], [3,0,19742.524000,19743.648000], [3,1,22413.841000,25925.919000], [3,3,25928.303000,25928.730000], [3,0,25930.247000,25932.484000], [3,0,26037.718000,26038.935000], [3,0,26200.338000,26210.509000], [3,0,26242.141000,26243.314000], [3,0,26363.216000,26364.239000], [3,0,26634.579000,26636.285000], [3,0,26710.827000,26712.225000], [3,0,26738.587000,26739.646000], [3,0,26874.154000,26880.452000], [3,0,26880.785000,26882.155000], [3,0,26909.591000,26911.692000], [3,0,26916.941000,27019.186000], [3,0,27019.392000,27022.665000], [3,0,27032.393000,27034.864000], [3,0,27078.693000,27079.711000], [3,0,27254.730000,27259.985000], [3,0,27469.312000,27485.273000], [3,0,27736.922000,27739.046000], [3,0,27802.646000,27803.649000], [3,0,27837.736000,27839.303000], [3,1,27888.081000,27890.231000], [3,3,27892.639000,27892.866000], 
	[4,0,7.607000,8.780000], [4,0,533.894000,534.925000], [4,0,1372.941000,1373.944000], [4,0,1963.862000,1964.917000], [4,0,2524.156000,2525.427000], [4,0,4729.284000,4745.847000], [4,0,4759.179000,4774.771000], [4,0,6596.061000,6597.086000], [4,0,7776.377000,7791.923000], [4,0,7878.697000,7894.620000], [4,0,8680.834000,8682.423000], [4,0,10686.715000,10687.730000], [4,0,13083.889000,13098.966000], [4,0,13745.136000,13746.195000], [4,0,14020.751000,14037.846000], [4,0,14190.316000,14206.385000], [4,0,15223.426000,15238.871000], [4,0,15794.272000,15795.470000], [4,0,17341.197000,17342.637000], [4,0,17347.235000,17348.539000], [4,0,17535.557000,17536.845000], [4,0,18108.787000,18109.973000], [4,0,18889.955000,18891.109000], [4,0,19250.453000,19251.466000], [4,1,22316.906000,25925.919000], [4,3,25927.827000,25928.730000], [4,0,26052.047000,26053.147000], [4,0,26202.182000,26211.222000], [4,0,26404.092000,26405.234000], [4,0,26436.621000,26438.082000], [4,0,26701.202000,26702.338000], [4,0,26745.458000,26746.644000], [4,0,26771.990000,26773.115000], [4,0,26834.606000,26835.806000], [4,0,26865.952000,26866.957000], [4,0,26874.409000,26881.747000], [4,0,26916.239000,27021.168000], [4,0,27021.416000,27023.319000], [4,0,27031.602000,27032.656000], [4,0,27230.202000,27231.294000], [4,0,27255.114000,27261.010000], [4,0,27261.297000,27262.857000], [4,0,27524.428000,27525.566000], [4,0,27652.535000,27653.688000], [4,0,27807.159000,27820.201000], [4,0,27858.568000,27859.584000], [4,1,27889.346000,27890.231000], [4,3,27892.569000,27892.866000], 
	[5,0,8.491000,9.573000], [5,0,10.100000,12.657000], [5,0,608.718000,609.842000], [5,0,1373.338000,1375.306000], [5,0,3936.193000,3951.796000], [5,0,4423.128000,4424.435000], [5,0,5569.893000,5586.511000], [5,0,6425.553000,6426.647000], [5,0,6683.013000,6684.036000], [5,0,6689.047000,6690.177000], [5,0,6701.963000,6717.038000], [5,0,7131.045000,7132.435000], [5,0,7678.960000,7680.293000], [5,0,7878.725000,7895.469000], [5,0,7911.728000,7913.097000], [5,0,9426.798000,9443.509000], [5,0,12190.502000,12206.051000], [5,0,12910.723000,12925.906000], [5,0,15953.356000,15955.285000], [5,0,16013.667000,16014.744000], [5,1,22537.100000,25925.919000], [5,3,25927.730000,25928.730000], [5,0,25930.243000,25931.345000], [5,0,26129.079000,26130.242000], [5,0,26199.607000,26209.721000], [5,0,26574.709000,26576.799000], [5,0,26581.422000,26582.865000], [5,0,26776.469000,26787.540000], [5,0,26787.739000,26789.344000], [5,0,26798.438000,26800.261000], [5,0,26874.620000,26879.685000], [5,0,26880.015000,26881.454000], [5,0,26911.328000,26912.378000], [5,0,26923.201000,27019.459000], [5,0,27046.840000,27048.039000], [5,0,27255.311000,27261.330000], [5,0,27284.013000,27285.024000], [5,0,27578.931000,27580.562000], [5,0,27807.318000,27818.568000], [5,1,27888.732000,27890.231000], [5,3,27892.578000,27892.866000], 
	[6,0,12.596000,15.121000], [6,0,4728.488000,4746.289000], [6,0,5569.960000,5586.488000], [6,0,5658.059000,5659.159000], [6,0,6288.417000,6289.505000], [6,0,6427.953000,6444.933000], [6,0,7167.779000,7184.225000], [6,0,7770.246000,7771.493000], [6,0,7912.004000,7913.436000], [6,0,7925.624000,7942.151000], [6,0,9426.808000,9443.628000], [6,0,11244.989000,11261.641000], [6,0,11953.447000,11955.019000], [6,0,12748.798000,12764.977000], [6,0,13345.406000,13361.521000], [6,0,13650.082000,13651.378000], [6,0,14021.057000,14037.137000], [6,0,16353.160000,16354.291000], [6,0,18611.790000,18612.813000], [6,0,21235.427000,21236.557000], [6,1,21780.860000,25925.919000], [6,3,25928.076000,25928.730000], [6,0,26010.702000,26011.836000], [6,0,26016.333000,26017.704000], [6,0,26178.659000,26179.686000], [6,0,26184.246000,26185.362000], [6,0,26199.176000,26208.966000], [6,0,26304.558000,26305.636000], [6,0,26332.909000,26334.338000], [6,0,26497.812000,26498.825000], [6,0,26560.940000,26562.149000], [6,0,26777.214000,26787.791000], [6,0,26869.121000,26870.376000], [6,0,26882.242000,26883.343000], [6,0,26908.317000,26909.850000], [6,0,26916.725000,27020.277000], [6,1,27889.956000,27890.231000], [6,3,27892.627000,27892.866000], 
	[7,0,8.163000,11.383000], [7,0,15.904000,17.058000], [7,0,19.566000,34.627000], [7,0,1839.683000,1841.422000], [7,0,2524.824000,2526.071000], [7,0,3349.630000,3350.656000], [7,0,3590.952000,3592.271000], [7,0,3936.158000,3951.787000], [7,0,4526.018000,4527.074000], [7,0,4759.248000,4774.848000], [7,0,4883.084000,4885.180000], [7,0,5539.645000,5540.647000], [7,0,5766.918000,5768.533000], [7,0,6578.852000,6579.968000], [7,0,6717.033000,6718.223000], [7,0,7167.762000,7184.154000], [7,0,7878.524000,7895.491000], [7,0,7911.621000,7914.270000], [7,0,7925.606000,7940.989000], [7,0,9156.566000,9157.818000], [7,0,9426.802000,9442.460000], [7,0,13443.206000,13444.481000], [7,0,13475.962000,13490.997000], [7,0,14036.418000,14037.785000], [7,0,14165.975000,14181.350000], [7,0,21151.040000,21152.161000], [7,0,21415.319000,21416.383000], [7,0,21426.563000,21427.613000], [7,1,25925.491000,25925.919000], [7,3,25927.800000,25928.730000], [7,0,25930.212000,25932.097000], [7,0,26069.581000,26070.958000], [7,0,26197.767000,26199.171000], [7,0,26501.888000,26503.056000], [7,0,26516.698000,26517.735000], [7,0,26570.584000,26571.745000], [7,0,26594.543000,26596.582000], [7,0,26610.868000,26611.957000], [7,0,26682.498000,26683.634000], [7,0,26775.980000,26788.064000], [7,0,26788.334000,26789.739000], [7,0,26845.411000,26847.191000], [7,0,26874.322000,26879.169000], [7,0,26913.783000,27019.754000], [7,0,27034.794000,27035.983000], [7,0,27077.344000,27078.764000], [7,0,27126.046000,27127.062000], [7,0,27134.679000,27135.814000], [7,0,27228.988000,27230.313000], [7,0,27255.685000,27260.496000], [7,0,27266.354000,27283.812000], [7,0,27633.881000,27634.887000], [7,0,27807.628000,27817.751000], [7,0,27818.004000,27820.926000], [7,1,27889.232000,27890.231000], [7,3,27892.513000,27892.866000], 
]
# Total, block, % = 27893.666000, 25295.755000, 90.686377 %
# Num nodes = 27784; 3274 (11.8 %), 3138 (11.3 %), 3512 (12.6 %), 3584 (12.9 %), 3700 (13.3 %), 3613 (13.0 %), 3747 (13.5 %), 3216 (11.6 %), 
TMAX = int(27893.666)

SCALE = 1000/TMAX

# Color Dict
tagColor = {
    0:"red", 
    1:"green", 
    2:"blue", 
    3:"purple", 
    4:"yellow",
    5:"orange",
}

# Get max time
if(TMAX<0):
    for datum in data:
        TMAX = int(datum[3]*SCALE) if int(datum[3]*SCALE)>TMAX else TMAX
TMAX = int(SCALE*TMAX)

# Set up frame
w,h = TMAX, HEIGHT*NUMPROCS
img = Image.new("RGB", (w,h))
img1 = ImageDraw.Draw(img)  
img1.rectangle([0,0,w,h], fill ='white')

# Set up timelines
for i in range(0,NUMPROCS):
    img1.rectangle([0,i*HEIGHT,TMAX,(i+1)*HEIGHT-MARGIN], fill ='white', outline='black')

# Add blocks
sumByTag = {}
sumTotal = 0
for datum in data:
    x1 = int(datum[2]*SCALE)
    y1 = HEIGHT * datum[0]
    x2 = int(datum[3]*SCALE)
    y2 = y1+HEIGHT-MARGIN
    img1.rectangle([x1,y1+1,x2,y2-1], fill =tagColor[datum[1]], outline =None)
    
    sumTotal += datum[3]-datum[2]
    if(datum[1] in sumByTag):
        sumByTag[datum[1]] += datum[3]-datum[2]
    else:
        sumByTag[datum[1]] = datum[3]-datum[2]

print('Blockage by tag:')
for key in sumByTag:
    print(f'\tTag {key}: {sumByTag[key]}, {100*sumByTag[key]/sumTotal}')

# Save
img.save('../outputs/timeline.png')
