# v2

# table_coord = {
# "pantry":[[12.791071,0.804028]],
# "pantry_left":[[14.658420,-1.037887]],
# "pantry_right":[[11.10,-0.809125]],
# "pantry_exit":[[13.083143,-0.262219],[13.024012,0.855195]],
# "meeting":[[8.691749,1.311024],[8.710741,2.242054]],
# "meeting_drop":[[6.862687,2.495237],[6.957338,2.577140]],
# "meeting_pick":[[7.969441,2.495237],[8.064076,2.680834]],
# "meeting_exit":[[8.064076,2.680834],[8.630914,1.924502],[8.775154,1.186096]],
# "researchM":[[10.603350,1.103348],[10.761787,10.068030],[11.621890,10.153056]],
# "researchM_exit":[[14.868692,10.135546]],
# "researchS":[[14.868692,10.135546],[11.621890,10.153056]],
# "researchS_exit":[[10.761787,10.068030],[10.713684,7.028123]],
# "store":[[16.075034,1.130147],[24.480119,-2.289210]],
# "store_left":[[26.098674,-2.765020]],
# "store_mid":[[26.098674,-2.765020],[25.789904,-3.338421]],
# "store_right":[[25.789904,-3.338421],[25.761535,-3.467302]], 
# "store_exit":[[24.695766,-2.217103],[14.786498,3.713265]],
# "conf":[[5.220025,1.026368],[5.249661,-0.028320],[5.589226,-0.633491]],
# "conf_exit":[[5.220025,1.026368],[5.249661,-0.028320],[5.589226,-0.633491]]
# }

# table_angles = {
# "pantry":[quaternion_from_euler(0,0,-1.145174)],
# "pantry_left":[quaternion_from_euler(0,0,-0.215539)],
# "pantry_right":[quaternion_from_euler(0,0,-3.136397)],
# "pantry_exit":[quaternion_from_euler(0,0,1.622419),quaternion_from_euler(0,0,1.618521)],
# "meeting":[quaternion_from_euler(0,0,1.558938),quaternion_from_euler(0,0,3.030832)],
# "meeting_drop":[quaternion_from_euler(0,0,3.14),quaternion_from_euler(0,0,0)],
# "meeting_pick":[quaternion_from_euler(0,0,-0.006798),quaternion_from_euler(0,0,1.562901)],
# "meeting_exit":[quaternion_from_euler(0,0,-1.016319),quaternion_from_euler(0,0,-1.351501),quaternion_from_euler(0,0,-0.022250)],
# "researchM":[quaternion_from_euler(0,0,1.590540),quaternion_from_euler(0,0,0.788266),quaternion_from_euler(0,0,-1.578276)],
# "researchM_exit":[quaternion_from_euler(0,0,-1.045800)],
# "researchS":[quaternion_from_euler(0,0,2.423048),quaternion_from_euler(0,0,-1.578276)],
# "researchS_exit":[quaternion_from_euler(0,0,-1.959147),quaternion_from_euler(0,0,-1.589767)],
# "store":[quaternion_from_euler(0,0,-0.593287),quaternion_from_euler(0,0,-0.563331)],
# "store_left":[quaternion_from_euler(0,0,-0.641048)],
# "store_mid":[quaternion_from_euler(0,0,-2.209354),quaternion_from_euler(0,0,-0.665531)],
# "store_right":[quaternion_from_euler(0,0,-2.209354),quaternion_from_euler(0,0,-0.665531)],
# "store_exit":[quaternion_from_euler(0,0,2.481335),quaternion_from_euler(0,0,-3.140157),quaternion_from_euler(0,0,-1.547272)],
# "conf":[quaternion_from_euler(0,0,-1.556558),quaternion_from_euler(0,0,-1.557076),quaternion_from_euler(0,0,-1.557076)],
# "conf_exit":[quaternion_from_euler(0,0,-1.556558),quaternion_from_euler(0,0,-1.557076)]
# }



# v1 

# # start -> pantry left
# [12.791071,0.804028],[14.629389,-1.011819]
# quaternion_from_euler(0,0,-1.145174),quaternion_from_euler(0,0,-0.008308)

# # pantry left -> pantry right
# [13.651953,-0.993823],[13.651953,-0.993823],[11.367226,-0.837971]
# quaternion_from_euler(0,0,-0.008308),quaternion_from_euler(0,0,3.112836),quaternion_from_euler(0,0,3.136623)

# # pantry -> meeting_drop
# [13.209870,-0.290010],[13.164483,0.129666],[13.128121,0.598994],[9.646137,1.093739],[8.736946,1.646485],[8.604864,2.101415],[8.144331,2.412496],[6.942131,2.537045]
# quaternion_from_euler(0,0,1.694734),quaternion_from_euler(0,0,1.683430),quaternion_from_euler(0,0,2.248923),quaternion_from_euler(0,0,3.112138),quaternion_from_euler(0,0,1.904479),quaternion_from_euler(0,0,1.827757),quaternion_from_euler(0,0,2.827783),quaternion_from_euler(0,0,-0.037878)

# # meeting_pick -> research lab
# [8.446069,2.356161],[8.650305,1.909871],[8.813283,1.330291],[9.701083,1.012474],[10.723454,7.464826],[10.799892,9.835673],[11.621146,10.129061]
# quaternion_from_euler(0,0,-1.195854),quaternion_from_euler(0,0,-1.515380),quaternion_from_euler(0,0,-0.904486),quaternion_from_euler(0,0,-0.004190),quaternion_from_euler(0,0,1.550717),quaternion_from_euler(0,0,1.065317),quaternion_from_euler(0,0,-1.564719)

# # research lab -> store left
# [14.051501,10.129633],[14.861696,10.034553],[15.073085,9.172235],[15.753784,3.616416],[26.099772,-2.851555]
# quaternion_from_euler(0,0,-0.007654),quaternion_from_euler(0,0,-0.725942),quaternion_from_euler(0,0,-1.571305),quaternion_from_euler(0,0,-0.460414),quaternion_from_euler(0,0,-0.625346)

# # store left -> store mid
# [26.099772,-2.851555],[25.987929,-3.017048],[25.884960,-3.184259],[25.893405,-3.193823]
# quaternion_from_euler(0,0,-0.625346),quaternion_from_euler(0,0,-2.203625),quaternion_from_euler(0,0,-2.192621),quaternion_from_euler(0,0,-0.641933)

# # store mid -> store right
# [25.893405,-3.193823],[25.711857,-3.466687]
# quaternion_from_euler(0,0,-2.203625),quaternion_from_euler(0,0,-0.625346)

# # store -> conf
# [15.933700,1.379293],[6.030317,0.915084],[5.534729,0.921002],[5.293755,0.266071],[5.302319,-0.036259],[5.594201,-0.621052]
# quaternion_from_euler(0,0,3.106431),quaternion_from_euler(0,0,3.137669),quaternion_from_euler(0,0,-2.013029),quaternion_from_euler(0,0,-1.644550),quaternion_from_euler(0,0,-1.581448),quaternion_from_euler(0,0,-1.580535)

# orig
# "meeting_drop":[[13.083143,-0.262219],[13.024012,0.855195],[8.691749,1.311024],[8.710741,2.242054],[6.862687,2.495237],
# "meeting_drop":[quaternion_from_euler(0,0,1.622419),quaternion_from_euler(0,0,1.618521),

# "research":[[8.064076,2.680834],[8.630914,1.924502],
# "research":[quaternion_from_euler(0,0,-1.016319),quaternion_from_euler(0,0,-1.351501),

# "store_left":[[14.868692,10.135546],
# "store_left":[quaternion_from_euler(0,0,-1.045800),