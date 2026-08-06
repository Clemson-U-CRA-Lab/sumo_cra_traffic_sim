### Ovarall TVT Aug 2026 R1 Revision Effort Data collection:

1. 2026_07_24: 

    - For TVT R1 revision
    - For benchmarking delay vs attack parmas (conducted by Soumil, overssen by Prakhar. Processed by Prakhar)
    - Processed csvs are renamed in the processed/ folder. 
    - pcapcs recorded on RSU, RSPC, OBU, Nuvo simultatenously for these runs

2. 2026_08_01:

    - This is continuation of benchamrking runs from 24 July 2026.

    - Needed more runs to plot the sensitivty analysis for TVT-R1 revision.

    - attack launched at 10sec.
    - pcaps recorded on obu, rsu only.


----> The above two files were used to complete sensitivty analysis plot (procsesing script is `.process_sensitivity_analysis.m` ).



3. 2026_08_05:

    - All the runs reran in SIM. (using "SAMEMACHINE" connection, `sumo2V_v5_nvN.py` & `test_dummies/dummy_samemachine_mpc_vehicle_variants.py` for all attack scenarios. Used `scripts/run_experiment_sim_sweep.py` to run batch).
    - Also ran the 3 vehicle scenarios now.. Added Fallback strategy to the sim mpc vehicle dummy to do this.
    - had to make sure trialing vehicles use the middle real CAV's intentions after recving it from UDP
    - The VEH_ARRAY_SIZE increased ot accomodat this.


4. 2026_08_06:

    - above runs, but on the actual car, Using "DIRECT" connection, and BOOL_TEST_WITHOUT_GPS = True
    - so the jitter and delays are slightly more variant and higher. 
    - Realistic.
    - Used `scripts/run_experiment_sim_sweep_hil.py` to launch batch runs [ONLY TO USE WHEN STATIONARY TESTS].

    - Use this data for final performane plots of TVT R1 revision (`.analysis_v6a.m`)