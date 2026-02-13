import numpy as np
from scipy.spatial.transform import Rotation as R
import os


def gravity_compute():
    """
    Compute the gravity compensation parameters from recorded data.

    This function reads recorded pose and wrench data from text files,
    computes the gravity compensation parameters using least squares,
    and saves the results to a text file.
    """
    # load the data from txt files
    home_dir = os.path.expanduser("~")
    gravity_comp_dir = os.path.join(home_dir, ".gravity_compensation")
    pose = np.loadtxt(gravity_comp_dir +
                      "/recorded_messages_pose.txt", delimiter=",")
    wrench = np.loadtxt(gravity_comp_dir +
                        "/recorded_messages_wrench.txt", delimiter=",")

    positions = pose[:, 0:3]
    quaternions = pose[:, 3:7]

    forces = wrench[:, 0:3]
    torques = wrench[:, 3:6]

    # The quaternions are the format of [x, y, z, w]
    rots = R.from_quat(quaternions)

    # compute the force part of the gravity compensation
    # Initialize the Least Squares Matrix
    # A*x = b
    # Which is:
    # A is a 3n x 4 matrix, where n is the number of data points;
    # x is a 4 x 1 vector, which is the gravity force and the preloaded force
    #    of the gripper;
    # b is the recored forces.
    # Each cell of 3x4 matrix on A is:
    # [[0,0,1]', rotation_matrix_3x3]
    # Because we have to part need to be computed:
    # 1. The force part of the gripper gravity. This part will changed with
    #    the gripper pose.
    # 2. The preloaded force of the gripper. This part will not changed with
    #    the gripper pose.
    # Due to we already have the transformed force data, so we set the force
    #    multipled [0, 0, 1] as the first column of A
    # Then we set the rotation matrix as the rest of the matrix to represent
    #    the preloaded force.

    # Initialize the A matrix
    A = np.zeros((3 * len(positions), 4))
    b = np.vstack(forces).reshape(-1, 1)
    for i in range(len(positions)):
        index = i * 3  # the index of the cell in the A matrix
        A[index + 2, 0] = 1
        A[index: index + 3, 1:] = rots[i].as_matrix()

    # Calculate the x by least squares
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    # The first element of x is the gravity force
    # The rest of the elements are the preloaded force of the gripper.
    # Check the error
    error = A @ x - b
    if error.max() > 1.0:
        print("The error is too large, please check the data!!!!!!!")

    # Compute the torque part of the gravity compensation
    # Initialize the Least Squares Matrix
    # The A is also a 3n x 4 matrix, where n is the number of data points;
    # x is a 4 x 1 vector, which the first element is the distance of the mass
    #    center to the gripper center along the z
    # The rest of the elements are the preloaded torque of the gripper which
    #    is not affected by poses.
    # b is the recorded torques.
    # Each cell of 3x4 matrix on A is little different from the force part:
    # 1. The first column is for the cross product matrix multiplied with z
    #    vector of the Rotation matrix as well as multiplied the gravity z
    #    force. : V = fg_z * Rz * CrossMatrix(z). size = 3x1
    # 2. The rest of the matrix is the rotation matrix. size = 3x3

    # Initialize the A matrix
    At = np.zeros((3 * len(positions), 4))
    bt = np.vstack(torques).reshape(-1, 1)
    cross_matrix = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 0]])
    for i in range(len(positions)):
        index = i * 3  # the index of the cell in the A matrix
        # the z component of the cross product matrix
        At[index: index + 3, 0] = x[0] * \
            (cross_matrix @ rots[i].as_matrix())[:, 2]
        At[index: index + 3, 1:] = rots[i].as_matrix()

    # Calculate the x by least squares
    xt = np.linalg.lstsq(At, bt, rcond=None)[0]
    # The first element of x is the distance of the mass center to the gripper
    #    center along the z
    # The rest of the elements are the preloaded torque of the gripper.
    # Check the error
    error_t = At @ xt - bt
    if error_t.max() > 0.1:
        print("The error is too large, please check the data!!!!!!!")

    # Save the result to the file as the format with specified float precision
    # Save the txt with string "force: [x, y, z], torque: [x, y, z]"
    with open(gravity_comp_dir + "/gravity_result.txt", "w") as f:
        x_list = [f"{i:.5f}" for i in x.reshape(-1).tolist()]
        xt_list = [f"{i:.5f}" for i in xt.reshape(-1).tolist()]
        f.write("force: \r\n")
        f.write(f"[{x_list[0]},{x_list[1]},{x_list[2]},{x_list[3]}]")
        f.write("\r\ntorque: \r\n")
        f.write(f"[{xt_list[0]},{xt_list[1]},{xt_list[2]},{xt_list[3]}]")
