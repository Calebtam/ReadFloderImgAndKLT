import subprocess

# Define file paths
# ground_truth_file = "ground_truth.txt"
ground_truth_file = "rtk.txt" 


# estimate_files = ["estimate1.txt", "estimate2.txt", "estimate3.txt", "estimate4.txt"]
estimate_files = ["imu-cam_imu-init.txt", "imu-cam_imu-wheel-init.txt", "imu-wheel-cam_imu-init.txt", "imu-wheel-cam_imu-wheel-init.txt"]

# Compare each estimate trajectory to the ground truth
for i, estimate_file in enumerate(estimate_files):
    # result_file = f"results{i+1}.txt"
    result_file = f"{estimate_file.replace('.txt','_ape')}.zip"
    # cmd = f"evo_ape tum {ground_truth_file} {estimate_file} -p --plot_mode=xy -a --n_to_align=100 -v --save_results_to_file {result_file}"
    cmd = f"evo_ape tum {ground_truth_file} {estimate_file} -p --plot_mode=xy -a --n_to_align=100 -v --save_results {result_file}"
    subprocess.run(cmd, shell=True)

    print(f"Comparison results for {estimate_file} saved to {result_file}")

# # Compare each estimate trajectory to the ground truth
# for i, estimate_file in enumerate(estimate_files):
    # result_file = f"results{i+1}.txt"    
    rpe_result_file = f"{estimate_file.replace('.txt','_rpe')}.zip"
    cmd = f"evo_rpe tum {ground_truth_file} {estimate_file} -r trans_part --plot --plot_mode xy -a --n_to_align=100 --project_to_plane=xy -v --save_results {rpe_result_file}"
    subprocess.run(cmd, shell=True)

    print(f"Comparison results for {estimate_file} saved to {rpe_result_file}")


print("Trajectory comparison completed!")
