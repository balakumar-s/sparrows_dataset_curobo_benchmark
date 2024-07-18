# Minimal Script to run cuRobo on SPARROWS dataset

1. Copy `no_filter_planning_results` from https://github.com/roahmlab/curobo/tree/main/no_filter_planning_results into `data/`
2. Copy `sparrows_comparison` from https://github.com/roahmlab/curobo/tree/main/src/curobo/content/configs/world/sparrows_comparison into `data/`
3. Install cuRobo from https://curobo.org .
4. Run `python comparison_for_sparrows.py` script to compute success and planning time.


## Results

|Number of Obstacles| Success (with 0.5 second time limit)| Planning Time(s)|
|-|-|-|
|10| 100 |0.039 |
|20| 98 |0.045 |
|40| 83 |0.060 |

Benchmark run on RTX 4090.

## References:
- [[SPARROWS](https://roahmlab.github.io/sparrows/)] Michaux, J., Li, A., Chen, Q., Chen, C., Zhang, B. and Vasudevan, R., 2024. Safe Planning for Articulated Robots Using Reachability-based Obstacle Avoidance With Spheres. arXiv preprint arXiv:2402.08857.

- [[cuRobo](https://curobo.org)] Sundaralingam, B., Hari, S.K.S., Fishman, A., Garrett, C., Van Wyk, K., Blukis, V., Millane, A., Oleynikova, H., Handa, A., Ramos, F. and Ratliff, N., 2023. CuRobo: Parallelized collision-free minimum-jerk robot motion generation. arXiv preprint arXiv:2310.17274.
