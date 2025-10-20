#!/usr/bin/env python3

"""
Performance Comparison Script
Compares Nav2 vs DQN navigation performance
"""

import json
import os
import sys
from pathlib import Path


def load_sessions(log_dir, mode):
    """Load session data from JSON file"""
    filepath = Path(log_dir) / f'all_sessions_{mode}.json'
    
    if not filepath.exists():
        print(f"Warning: {filepath} not found")
        return []
    
    with open(filepath, 'r') as f:
        return json.load(f)


def calculate_stats(sessions):
    """Calculate statistics from sessions"""
    if not sessions:
        return None
    
    successful = [s for s in sessions if s.get('success', False)]
    
    if not successful:
        return {
            'total_sessions': len(sessions),
            'success_rate': 0.0,
            'avg_duration': 0.0,
            'avg_distance': 0.0,
            'avg_efficiency': 0.0,
            'total_collisions': sum(s.get('collision_count', 0) for s in sessions)
        }
    
    return {
        'total_sessions': len(sessions),
        'successful_sessions': len(successful),
        'success_rate': len(successful) / len(sessions) * 100,
        'avg_duration': sum(s['duration_seconds'] for s in successful) / len(successful),
        'avg_distance': sum(s['total_distance_traveled'] for s in successful) / len(successful),
        'avg_efficiency': sum(s['path_efficiency'] for s in successful) / len(successful),
        'avg_speed': sum(s.get('average_speed', 0) for s in successful) / len(successful),
        'total_collisions': sum(s.get('collision_count', 0) for s in sessions),
        'avg_collisions': sum(s.get('collision_count', 0) for s in sessions) / len(sessions)
    }


def print_stats(mode, stats):
    """Print statistics in formatted manner"""
    if stats is None:
        print(f"\n{mode.upper()}: No data available")
        return
    
    print(f"\n{'='*50}")
    print(f"  {mode.upper()} Performance Statistics")
    print(f"{'='*50}")
    print(f"Total Sessions:      {stats['total_sessions']}")
    print(f"Successful:          {stats.get('successful_sessions', 0)}")
    print(f"Success Rate:        {stats['success_rate']:.1f}%")
    print(f"Avg Duration:        {stats['avg_duration']:.2f} seconds")
    print(f"Avg Distance:        {stats['avg_distance']:.2f} meters")
    print(f"Avg Path Efficiency: {stats['avg_efficiency']:.1f}%")
    print(f"Avg Speed:           {stats.get('avg_speed', 0):.2f} m/s")
    print(f"Total Collisions:    {stats['total_collisions']}")
    print(f"Avg Collisions:      {stats.get('avg_collisions', 0):.2f} per session")
    print(f"{'='*50}")


def compare_modes(nav2_stats, dqn_stats):
    """Compare Nav2 and DQN statistics"""
    print(f"\n{'='*50}")
    print(f"  Comparison (Nav2 vs DQN)")
    print(f"{'='*50}")
    
    if nav2_stats and dqn_stats:
        metrics = [
            ('Success Rate', 'success_rate', '%', 'higher'),
            ('Avg Duration', 'avg_duration', 's', 'lower'),
            ('Avg Distance', 'avg_distance', 'm', 'lower'),
            ('Path Efficiency', 'avg_efficiency', '%', 'higher'),
            ('Avg Collisions', 'avg_collisions', '', 'lower')
        ]
        
        for name, key, unit, better in metrics:
            nav2_val = nav2_stats.get(key, 0)
            dqn_val = dqn_stats.get(key, 0)
            
            if nav2_val == 0 and dqn_val == 0:
                continue
            
            diff = dqn_val - nav2_val
            percent = (diff / nav2_val * 100) if nav2_val != 0 else 0
            
            if better == 'higher':
                winner = "DQN" if dqn_val > nav2_val else "Nav2"
                symbol = "↑" if dqn_val > nav2_val else "↓"
            else:
                winner = "DQN" if dqn_val < nav2_val else "Nav2"
                symbol = "↓" if dqn_val < nav2_val else "↑"
            
            print(f"{name:20s}: Nav2={nav2_val:7.2f}{unit:3s} | "
                  f"DQN={dqn_val:7.2f}{unit:3s} | "
                  f"Diff={diff:+7.2f} ({percent:+6.1f}%) {symbol} Winner: {winner}")
    
    print(f"{'='*50}\n")


def main():
    """Main function"""
    log_dir = './navigation_logs'
    
    if len(sys.argv) > 1:
        log_dir = sys.argv[1]
    
    if not os.path.exists(log_dir):
        print(f"Error: Log directory '{log_dir}' not found")
        print("Usage: python3 compare_performance.py [log_directory]")
        sys.exit(1)
    
    print("Loading session data...")
    nav2_sessions = load_sessions(log_dir, 'nav2')
    dqn_sessions = load_sessions(log_dir, 'dqn')
    
    nav2_stats = calculate_stats(nav2_sessions)
    dqn_stats = calculate_stats(dqn_sessions)
    
    print_stats('Nav2', nav2_stats)
    print_stats('DQN', dqn_stats)
    
    if nav2_stats and dqn_stats:
        compare_modes(nav2_stats, dqn_stats)
    else:
        print("\nInsufficient data for comparison.")
        print("Run navigation tests with both Nav2 and DQN modes first.")


if __name__ == '__main__':
    main()

