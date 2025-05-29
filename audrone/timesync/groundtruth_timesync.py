import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import correlate
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

def quaternion_to_angular_velocity(quaternions, dt):
    """
    Convert quaternion sequence to angular velocities.
    
    Args:
        quaternions: Array of quaternions [N, 4] in format [x, y, z, w]
        dt: Time step between quaternions
    
    Returns:
        angular_velocities: Array of angular velocities [N-1, 3]
    """
    angular_velocities = []
    
    for i in range(len(quaternions) - 1):
        q1 = quaternions[i]
        q2 = quaternions[i + 1]
        
        # Create rotation objects
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        
        # Calculate relative rotation
        r_rel = r2 * r1.inv()
        
        # Convert to axis-angle representation
        rotvec = r_rel.as_rotvec()
        
        # Angular velocity is rotation vector divided by time step
        angular_vel = rotvec / dt
        angular_velocities.append(angular_vel)
    
    return np.array(angular_velocities)

def position_to_acceleration(positions, dt):
    """
    Convert position sequence to linear accelerations using numerical differentiation.
    
    Args:
        positions: Array of positions [N, 3]
        dt: Time step between positions
    
    Returns:
        accelerations: Array of accelerations [N-2, 3]
    """
    # First derivative (velocity)
    velocities = np.diff(positions, axis=0) / dt
    
    # Second derivative (acceleration)
    accelerations = np.diff(velocities, axis=0) / dt
    
    return accelerations

def cross_correlate_signals(signal1, signal2, max_lag=None):
    """
    Cross-correlate two signals and find the best alignment.
    
    Args:
        signal1: Reference signal
        signal2: Signal to align
        max_lag: Maximum lag to consider (in samples)
    
    Returns:
        lag: Best lag in samples (positive means signal2 is delayed)
        correlation: Cross-correlation values
        lags: Lag values corresponding to correlation
    """
    # Ensure signals are 1D for correlation
    if signal1.ndim > 1:
        signal1 = np.linalg.norm(signal1, axis=1)
    if signal2.ndim > 1:
        signal2 = np.linalg.norm(signal2, axis=1)
    
    # Normalize signals
    signal1 = (signal1 - np.mean(signal1)) / np.std(signal1)
    signal2 = (signal2 - np.mean(signal2)) / np.std(signal2)
    
    # Perform cross-correlation
    correlation = correlate(signal1, signal2, mode='full')
    lags = np.arange(-len(signal2) + 1, len(signal1))
    
    # Limit to max_lag if specified
    if max_lag is not None:
        valid_indices = np.abs(lags) <= max_lag
        correlation = correlation[valid_indices]
        lags = lags[valid_indices]
    
    # Find the lag with maximum correlation
    best_lag_idx = np.argmax(correlation)
    best_lag = lags[best_lag_idx]
    
    return best_lag, correlation, lags

def load_and_process_data(groundtruth_file, imu_file):
    """
    Load and process groundtruth and IMU data.
    
    Args:
        groundtruth_file: Path to groundtruth CSV file
        imu_file: Path to IMU CSV file
    
    Returns:
        Processed data dictionary
    """
    # Load groundtruth data
    gt_data = pd.read_csv(groundtruth_file)
    gt_columns = ['Frame', 'blank', 'RX', 'RY', 'RZ', 'RW', 'TX', 'TY', 'TZ']
    gt_data.columns = gt_columns
    
    # Load IMU data
    imu_data = pd.read_csv(imu_file)
    imu_columns = ['timestamp_ns', 'w_x', 'w_y', 'w_z', 'a_x', 'a_y', 'a_z']
    imu_data.columns = imu_columns
    
    # Extract groundtruth quaternions and positions
    gt_quaternions = gt_data[['RX', 'RY', 'RZ', 'RW']].values
    gt_positions = gt_data[['TX', 'TY', 'TZ']].values
    
    # Extract IMU data
    imu_timestamps = imu_data['timestamp_ns'].values
    imu_angular_vel = imu_data[['w_x', 'w_y', 'w_z']].values
    imu_linear_acc = imu_data[['a_x', 'a_y', 'a_z']].values
    
    # Convert timestamps to seconds (assuming first timestamp is t=0)
    imu_timestamps_sec = (imu_timestamps - imu_timestamps[0]) * 1e-9
    
    # Groundtruth sampling rate
    gt_dt = 1.0 / 100.0  # 100 Hz
    gt_timestamps_sec = np.arange(len(gt_data)) * gt_dt
    
    # Convert groundtruth to angular velocities and accelerations
    gt_angular_vel = quaternion_to_angular_velocity(gt_quaternions, gt_dt)
    gt_linear_acc = position_to_acceleration(gt_positions, gt_dt)
    
    # Adjust timestamps for differentiated signals
    gt_angular_vel_timestamps = gt_timestamps_sec[:-1]  # One less due to differentiation
    gt_linear_acc_timestamps = gt_timestamps_sec[:-2]   # Two less due to double differentiation
    
    return {
        'gt_angular_vel': gt_angular_vel,
        'gt_linear_acc': gt_linear_acc,
        'gt_angular_vel_timestamps': gt_angular_vel_timestamps,
        'gt_linear_acc_timestamps': gt_linear_acc_timestamps,
        'imu_angular_vel': imu_angular_vel,
        'imu_linear_acc': imu_linear_acc,
        'imu_timestamps': imu_timestamps_sec
    }

def find_time_offset(groundtruth_file, imu_file, max_lag_seconds=5.0, plot_results=True):
    """
    Find time offset between groundtruth and IMU data.
    
    Args:
        groundtruth_file: Path to groundtruth CSV file
        imu_file: Path to IMU CSV file
        max_lag_seconds: Maximum lag to consider in seconds
        plot_results: Whether to plot correlation results
    
    Returns:
        Dictionary with offset results
    """
    # Load and process data
    data = load_and_process_data(groundtruth_file, imu_file)
    
    # Interpolate groundtruth data to IMU timestamps for comparison
    # This helps with different sampling rates
    
    # For angular velocity
    interp_func_w = interp1d(
        data['gt_angular_vel_timestamps'], 
        data['gt_angular_vel'], 
        axis=0, 
        kind='linear', 
        bounds_error=False, 
        fill_value='extrapolate'
    )
    gt_angular_vel_interp = interp_func_w(data['imu_timestamps'])
    
    # For linear acceleration
    interp_func_a = interp1d(
        data['gt_linear_acc_timestamps'], 
        data['gt_linear_acc'], 
        axis=0, 
        kind='linear', 
        bounds_error=False, 
        fill_value='extrapolate'
    )
    gt_linear_acc_interp = interp_func_a(data['imu_timestamps'])
    
    # Calculate IMU sampling rate
    imu_dt = np.mean(np.diff(data['imu_timestamps']))
    max_lag_samples = int(max_lag_seconds / imu_dt)
    
    # Cross-correlate angular velocities
    lag_w, corr_w, lags_w = cross_correlate_signals(
        gt_angular_vel_interp, 
        data['imu_angular_vel'], 
        max_lag_samples
    )
    
    # Cross-correlate linear accelerations
    lag_a, corr_a, lags_a = cross_correlate_signals(
        gt_linear_acc_interp, 
        data['imu_linear_acc'], 
        max_lag_samples
    )
    
    # Convert lags to time
    lag_w_time = lag_w * imu_dt
    lag_a_time = lag_a * imu_dt
    lags_w_time = lags_w * imu_dt
    lags_a_time = lags_a * imu_dt
    
    # Results
    results = {
        'angular_velocity_offset_seconds': lag_w_time,
        'linear_acceleration_offset_seconds': lag_a_time,
        'angular_velocity_correlation': np.max(corr_w),
        'linear_acceleration_correlation': np.max(corr_a),
        'imu_sampling_rate': 1.0 / imu_dt,
        'correlation_data': {
            'angular_vel': {'lags': lags_w_time, 'correlation': corr_w},
            'linear_acc': {'lags': lags_a_time, 'correlation': corr_a}
        }
    }
    
    if plot_results:
        plot_correlation_results(results)
    
    return results

def plot_correlation_results(results):
    """Plot cross-correlation results."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Angular velocity correlation
    ax1.plot(
        results['correlation_data']['angular_vel']['lags'], 
        results['correlation_data']['angular_vel']['correlation']
    )
    ax1.axvline(
        results['angular_velocity_offset_seconds'], 
        color='red', 
        linestyle='--', 
        label=f'Best offset: {results["angular_velocity_offset_seconds"]:.3f}s'
    )
    ax1.set_xlabel('Time Offset (seconds)')
    ax1.set_ylabel('Cross-correlation')
    ax1.set_title('Angular Velocity Cross-correlation')
    ax1.legend()
    ax1.grid(True)
    
    # Linear acceleration correlation
    ax2.plot(
        results['correlation_data']['linear_acc']['lags'], 
        results['correlation_data']['linear_acc']['correlation']
    )
    ax2.axvline(
        results['linear_acceleration_offset_seconds'], 
        color='red', 
        linestyle='--', 
        label=f'Best offset: {results["linear_acceleration_offset_seconds"]:.3f}s'
    )
    ax2.set_xlabel('Time Offset (seconds)')
    ax2.set_ylabel('Cross-correlation')
    ax2.set_title('Linear Acceleration Cross-correlation')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    # Replace with your actual file paths
    groundtruth_file = "groundtruth_log6.csv"
    imu_file = "imu_data_log6.csv"

    try:
        # Find time offset
        results = find_time_offset(groundtruth_file, imu_file, max_lag_seconds=1.5)
        
        print("Time Synchronization Results:")
        print(f"Angular Velocity Offset: {results['angular_velocity_offset_seconds']:.4f} seconds")
        print(f"Linear Acceleration Offset: {results['linear_acceleration_offset_seconds']:.4f} seconds")
        print(f"Angular Velocity Max Correlation: {results['angular_velocity_correlation']:.4f}")
        print(f"Linear Acceleration Max Correlation: {results['linear_acceleration_correlation']:.4f}")
        print(f"IMU Sampling Rate: {results['imu_sampling_rate']:.2f} Hz")
        
        # Average offset (you might want to use one or the other depending on your application)
        avg_offset = (results['angular_velocity_offset_seconds'] + results['linear_acceleration_offset_seconds']) / 2
        print(f"Average Offset: {avg_offset:.4f} seconds")
        
        # Interpretation
        if avg_offset > 0:
            print("IMU data is delayed relative to groundtruth")
        else:
            print("IMU data is ahead relative to groundtruth")
            
    except FileNotFoundError as e:
        print(f"Error: Could not find file - {e}")
        print("Please make sure the CSV files exist and the paths are correct.")
    except Exception as e:
        print(f"Error processing data: {e}")