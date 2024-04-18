import numpy as np
import pandas as pd
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from sklearn.preprocessing import StandardScaler

def random_ik_parameters(seed=None):
    """
    Generate random inverse kinematics parameters.
    """
    np.random.seed(seed)
    return np.random.rand(3) * 3  # Random joint angles between 0 and 2*pi

def generate_robotic_arm_solutions(num_solutions, num_joints):
    """
    Generate random robotic arm solutions and save them to a CSV file.
    """
    # Create a URDF file for a 6-DOF robot arm
    urdf_filename = "arm_urdfs/ur5e_with_gripper.urdf"  # Provide the correct path to your URDF file
    arm = Chain.from_urdf_file(urdf_filename)

    # Initialize DataFrame to store solutions
    
    df_list = []  # List to store individual DataFrames

    # Generate and save random solutions
    for i in range(num_solutions):
        # Generate a random seed for reproducibility
        seed = np.random.randint(1, num_solutions)

        # Generate random inverse kinematics parameters
        task_space_position = random_ik_parameters()
        joint_angles = arm.inverse_kinematics(task_space_position)

        # print(task_space_position)

        # Create a DataFrame for the current solution
        data = {
            'Joint_Angles': [joint_angles.tolist()],
            'Task_Space_Position': [task_space_position.tolist()],
            'Seed': [seed]
        }
        df_list.append(pd.DataFrame(data))

    # Concatenate all DataFrames into one
    df = pd.concat(df_list, ignore_index=True)

    # Save DataFrame to CSV file
    df.to_csv('data/robotic_arm_solutions.csv', index=False)

    print("Data saved to 'data/robotic_arm_solutions.csv'.")

def main():
    # Define the robotic arm parameters
    num_joints = 8
    num_solutions = 1000

    # Generate and save robotic arm solutions
    generate_robotic_arm_solutions(num_solutions, num_joints)



if __name__ == "__main__":
    main()
