Note to configure the correct Sysconfig file based on the board revision
-------------------------------------------

This example provides two syscfg files to support different board revisions:
1. example.syscfg - For RevA boards (default in SDK 11.00+)
2. rev_e2_example.syscfg - For RevE2 boards (backward compatibility)

How to select the correct syscfg file:
-------------------------------------

1. Check your board revision mark (RevA or RevE2).

2. For RevA boards:
   - Use the default example.syscfg file.
   - No additional changes required.
   - Build the application.

3. For RevE2 boards:
   - Rename rev_e2_example.syscfg to example.syscfg
   - Build the application.

Key differences between configurations:
-------------------------------------
- RevA boards use DP83869 PHYs (default in SDK 11.00+)
- RevE2 boards use DP83826E PHYs
- PHY addresses differ between board revisions:
  * RevA: phyAddr0=3, phyAddr1=12
  * RevE2: phyAddr0=1, phyAddr1=3

Note: Using the wrong syscfg file for your board revision will result in initialization failures.