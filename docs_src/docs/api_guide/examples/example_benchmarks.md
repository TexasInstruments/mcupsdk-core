#  Benchmarks {#EXAMPLES_BENCHMARKS}

This page lists all the examples related to benchmarks of SOC and board peripheral drivers.

\cond SOC_AM64X || SOC_AM243X
   -# \subpage EXAMPLE_BENCHMARKDEMO
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM243X
   -# \subpage BENCHMARK_SMART_PLACEMENT
\endcond

\cond  SOC_AM263X || SOC_AM263PX || SOC_AM64X || SOC_AM243X || SOC_AM273X
   -# \subpage EXAMPLES_COREMARK
   -# \subpage EXAMPLES_DHRYSTONE
\endcond

\cond SOC_AM263X || SOC_AM263PX
   -# \subpage EXAMPLES_MEMORY_ACCESS_LATENCY
\endcond
