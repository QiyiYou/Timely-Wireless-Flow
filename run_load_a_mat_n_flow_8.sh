for (( i=8; i<=8; i++ ))
do
    #for n in 59 246 216 150 144 123 9 58 30 292 260 96     #cic20
    #for n in 95 93 92 89 88 85 81 7 78 77 76 62 5 56 53    #cic21
	#for n in 49 48 3 37 31 309 301 29 298 297 291 287 284  #cic22
	#for n in 280 274 273 272 271 263 262 257 256 250 24    #cic23
	#for n in 249 248 244 237 236 235 22 223 222 220 215    #cic24
	#for n in 213 212 211 210 20 206 205 197 196 195 193    #cic25
	#for n in 190 18 189 188 182 17 173 172 171 164 163     #cic26
	#for n in 15 158 152 151 149 147 143 139 133 132 12     #cic27 
	#for n in 126 121 118 116 10 104 102 98 94 87 86 83     #cic28
	#for n in 80 74 6 68 63 60 57 54 47 46 45 43 42 41      #cic29
	#for n in 39 38 36 35 33 315 312 310 306 305 302 290    #cic30
	#for n in 285 281 279 275 270 269 267 266 265 264 261   #cic31
	#for n in 259 258 255 251 242 241 240 23 239 238 233    #cic32
	#for n in 230 225 224 221 21 219 218 209 207 203 202    #not run now   
	#for n in 201 200 192 191 187 186 183 181 179 177 176   #not run now
	#for n in 175 174 16 166 162 161 160 159 155 154 153    #not run now
	#for n in 148 142 13 138 137 129 122 119 117 113 111    #not run now
	#for n in 108 107 106 8 75 51 32 2 28 282 276 253 247   #not run now
	#for n in 243 229 165 140 124 254 217 208 103           #not run now
    do
        sed -i "s/n_flow=[0-9].*/n_flow=$i/" comparision_scheduling_algorithm_many_flows_load_a_mat.m
        sed -i "s/next_conf=[0-9].*/next_conf=$n/" comparision_scheduling_algorithm_many_flows_load_a_mat.m
        sed -i 's/n_rate_instance=[0-9].*/n_rate_instance=100/' comparision_scheduling_algorithm_many_flows_load_a_mat.m
        sed -i 's/T=[0-9].*/T=10000/' comparision_scheduling_algorithm_many_flows_load_a_mat.m
        matlab < comparision_scheduling_algorithm_many_flows_load_a_mat.m &> run_load_a_mat_${i}_${n}.log
    done
done
