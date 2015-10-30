 1506  make -j5 && ./pod-build/bin/testBoundsR3 
 1508  make -j5 && ./pod-build/bin/testBoundsR3 
 1514  make -j5 && ./pod-build/bin/testBranchAndBoundR3
 1518  make -j5 && ./pod-build/bin/testBranchAndBoundR3
 1522  make -j5 && ./pod-build/bin/testBranchAndBoundR3
 1523  time ./pod-build/bin/testBranchAndBoundR3
 1561  ./pod-build/bin/testBoundsR3 
 1562  make && ./pod-build/bin/testBoundsR3 
 1563  make && ./pod-build/bin/testBranchAndBoundR3 
 1589  ./pod-build/bin/dpvMFoptRotPly -h
 1590  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply
 1591  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply
 1592  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60
 1598  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60
 1599  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1600  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60
 1601  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1602  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1608  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1610  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/right.ply -b ./data/middle.ply -l 60 -d
 1611  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left -l 60 -d
 1612  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1655  ./pod-build/bin/testBranchAndBound 
 1657  ./pod-build/bin/testBranchAndBound 
 1686  ./pod-build/bin/testBranchAndBoundR3 
 1687  gdb --args ./pod-build/bin/testBranchAndBoundR3 
 1689  make -j5 && gdb --args ./pod-build/bin/testBranchAndBoundR3 
 1706  git rm pod-build/ 
 1707  git rm -rf pod-build/ 
 1769  make -j5 && ./pod-build/bin/testBranchAndBoundR3
 1787  make -j5 && ./pod-build/bin/testBranchAndBoundT3
 1789  make -j5 && ./pod-build/bin/testBranchAndBoundT3
 1791  make -j5 && ./pod-build/bin/testBranchAndBoundT3
 1809  ./pod-build/bin/testBranchAndBoundT3 
 1812  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1814  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1816  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1821  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1823  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1824  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1825  make -j4 && ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -d
 1930  ./pod-build/bin/testBoundsR3 
 1931  make -j5 && ./pod-build/bin/testBranchAndBoundR3
 1932  make -j5 && ./pod-build/bin/testBoundsR3 
 1940  make -j5 && ./pod-build/bin/testBoundsR3
 1943  make -j5 && ./pod-build/bin/testBoundsR3
 2080  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 --d 
 2081  /pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 6-t 0.1 -d 
 2082  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 6-t 0.1 -d 
 2083  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.1 -d 
 2084  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.01 -d 
 2085  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.0i01 -d 
 2086  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.001 -d 
 2088  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 60 -t 0.001 -d 
 2090  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 60 -t 0.001 -d 
 2092  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 60 -t 0.001 -d 
 2093  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.001 -d 
 2094  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.01 -d 
 2095  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 3 -t 0.01 -d 
 2096  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20 -t 0.01 -d 
 2098  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20 -t 0.01 -d 
 2099  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.001 -d 
 2108  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 10 -t 0.001 -d 
 2109  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 20 -t 0.001 -d 
 2110  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 5. -t 0.0001 -d 
 2112  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 5. -t 0.0001 -d 
 2113  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 5. -t 0.0001 -d 
 2114  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 5. -t 0.001 -d 
 2115  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 10 -t 0.001 -d 
 2116  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.5 -d
 2117  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.5 -d
 2118* ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.kk1 -d
 2119  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.1 -d
 2120  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.2 -d
 2121  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.4 -d
 2122  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2123  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d | less
 2124  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d 
 2125  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.2 -d 
 2126  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d 
 2127  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.4 -d
 2128  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/right.ply -l 60 -t 0.4 -d
 2129  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/right.ply -l 30 -t 0.2 -d
 2130  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/right.ply -l 30 -t 0.3 -d
 2131  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 10 -t 0.001 -d
 2132  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045.ply -l 10 -t 0.001 -d
 2133  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.001 -d
 2134  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.001 -d -s 100.
 2135  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 30 -t 0.005 -d -s 10.
 2136  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2137  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.4 -d
 2143  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.4 -d
 2144  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 50 -t 0.2 -d
 2145  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2146  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun000_normals_angle_30_translation_0.1.ply -l 50 -t 0.005 -d -s 10.
 2147  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2148  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.005 -d -s 100.
 2149  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2150  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d | less
 2151  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2152  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 50 -t 0.2 -d
 2153  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.2 -d
 2154  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2157  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2180  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2181  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2182  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 50 -t 0.005 -d -s 10.
 2183  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/right.ply -l 60 -t 0.4 -d
 2184  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10 -t 0.005 -d 
 2185  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 5. -t 0.005 -d 
 2186  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun090_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 5. -t 0.005 -d 
 2187  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 20. -t 0.005 -d 
 2188  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 20. -t 0.002 -d 
 2189  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2190  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 20. -t 0.002 -d 
 2191  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20. -t 0.005 -d
 2192  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20. -t 0.002 -d
 2193  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2194  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 20. -t 0.002 -d 
 2195  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20. -t 0.002 -d
 2196  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 30. -t 0.002 -d
 2197  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10. -t 0.002 -d
 2198  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 10. -t 0.001 -d
 2199  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 1. -t 0.001 -d
 2200  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20. -t 0.001 -d
 2201  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 30. -t 0.001 -d
 2202  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 40. -t 0.001 -d
 2203  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 30. -t 0.001 -d
 2204  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 20. -t 0.001 -d
 2205  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000_normals.ply -b ./data/bunny/data/bun045_normals.ply -l 40. -t 0.001 -d
 2206  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 20. -t 0.002 -d 
 2235  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2237  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2239  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2241  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2243  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2245  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2246  make -j5 && ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2248  make -j5 && ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2249  make -j5 icp_T3 && ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2250  make -j5 icp_T3 && ./pod-build/bin/icp_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2251  make -j5 icp_T3 && ./pod-build/bin/icp_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -t ./data/middle_angle_30_translation_1_TrueTransformation.csv 
 2252  make -j5 icp_T3 && ./pod-build/bin/icp_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2253  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2255  make -j5 && ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2257  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply 
 2258  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -d
 2259  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -d -o middle_angle_30_translation_1_MM
 2261  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -d 
 2263  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -d 
 2264  ./pod-build/bin/moment_matched_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -d -o middle_angle_30_translation_1_MM
 2265  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -d -o middle_angle_30_translation_1_BB
 2266  ./pod-build/bin/icp_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -t middle_angle_30_translation_1_MM.csv
 2267  ./pod-build/bin/icp_T3 -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -t middle_angle_30_translation_1_MM.csv -d
 2268  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d -o bun000_045
 2269  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d -t bun000_045.csv 
 2270  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d 
 2271  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.05 -d -o bun000_045
 2274  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.05 -d -o bun000_045
 2275  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.005 -d -o bun000_045
 2276  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d
 2277  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d -t bun000_045.csv 
 2278  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d
 2279  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -d
 2280  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -l 30 -t 0.005 -d -o bun000_090
 2281  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.005
 2294  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.005
 2295  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.005 -d
 2296  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.001 -d
 2298  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.001 -d
 2300  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 30 -t 0.001 -d
 2301  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.001 -d
 2302  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d
 2303  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 90 -t 0.003 -d
 2304  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d
 2305  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 90 -t 0.003 -d
 2306  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d
 2307  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -o bun000_045
 2308  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045.csv 
 2309  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045.csv -d
 2310  ./pod-build/bin/moment_matched_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply  -o bun000_045_MM
 2311  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_MM.csv -d
 2312  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d
 2313  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -t bun000_045.csv -d
 2314  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply  -d
 2315  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -n -d
 2316  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -n a -d
 2317  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -l 60 -t 0.003 -o bun000_090
 2318  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -l 60 -t 0.003 -o bun000_090 -d
 2319  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun090.ply -l 30 -t 0.003 -o bun000_090 -d
 2320  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 60 -t 0.003 -o happySideRight_0_24 -d
 2321  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 40 -t 0.003 -o happySideRight_0_24 -d
 2322  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -l 40 -t 0.001 -o happySideRight_0_24 -d
 2323  ./pod-build/bin/icp_T3 -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -t happySideRight_0_24.csv -d
 2324  ./pod-build/bin/icp_T3 -a ./data/happy_side/happySideRight_0.ply -b ./data/happy_side/happySideRight_24.ply -d
 2330  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/right.ply -l 60 -t 0.4 -d
 2331  gdb --args ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/right.ply -l 60 -t 0.4 -d
 2332  gdb --args ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/left.ply -l 60 -t 0.4 -d
 2333  ./pod-build/bin/dpvMFoptRotPly -a ./data/middle.ply -b ./data/middle_angle_30_translation_1.ply -l 60 -t 0.4 -d
 2334  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d
 2335  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d -o bun000_045_BB
 2336  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_BB
 2337  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_BB.csv 
 2338  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_BB.csv  -d
 2339  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -d
 2340  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_BB.csv  -d
 2342  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_BB.csv  -d
 2343  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d
 2344  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side_rnd/happySideRight_48_angle_90_translation_0.3.ply -b ./data/happy_side_rnd/happySideRight_96_angle_90_translation_0.3.ply -l 60 -t 0.001
 2345  ./pod-build/bin/icp_T3 -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -t bun000_045_BB.csv  -d
 2346  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side_rnd/happySideRight_48_angle_90_translation_0.3.ply -b ./data/happy_side_rnd/happySideRight_96_angle_90_translation_0.3.ply -l 60 -t 0.001
 2347  ./pod-build/bin/dpvMFoptRotPly -a ./data/happy_side_rnd/happySideRight_48_angle_90_translation_0.3.ply -b ./data/happy_side_rnd/happySideRight_96_angle_90_translation_0.3.ply -l 60 -t 0.001 -d
 2356  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 -d 
 2360  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 
 2363  ./pod-build/bin/dpvMFoptRotPly -a ./data/bunny/data/bun000.ply -b ./data/bunny/data/bun045.ply -l 60 -t 0.003 
 2365  history  | grep pod-build
 2366  history  | grep pod-build > historic_cmds.sh
