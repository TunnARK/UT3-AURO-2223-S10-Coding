import numpy as np
import icp
import datatools
import matplotlib.pyplot as plt


if __name__ == "__main__":
   
    # Load pre-processed model point cloud
    print("Extracting MODEL object...")
    model = datatools.load_XYZ_data_to_vec('data/data01_segmented.xyz')
    
    # Load raw data point cloud
    print("Extracting DATA02 object...")
    data02_object = datatools.load_XYZ_data_to_vec('data/data02_object.xyz')
    
    # Load raw data point cloud
    print("Extracting DATA03 object...")
    data03_object = datatools.load_XYZ_data_to_vec('data/data03_object.xyz')

    # Make modelH a homogeneous representation of model
    model_object = np.ones((model.shape[0], 4))
    model_object[:,0:3] = np.copy(model)

    # Apply initial rotation to model point cloud
    ''' 
    theta = np.radians(36)
    c, s = np.cos(theta), np.sin(theta)
    rotation_matrix_x = np.array(((1, 0, 0, 0),(0, c, s, 0),(0, -s, c, 0),(0, 0, 0, 1)))
    model_object = np.dot(rotation_matrix_x, model_object.T).T
    model_object = np.delete(model_object, 3, 1)
    '''

    ref = model
    data = data02_object
#    data = data03_object
    
    print('Reference size : '+str(ref.shape))
    print('Raw data  size : '+str(data.shape))
    
    ##########################################################################
    # Run ICP to get data transformation w.r.t the model, final error and execution time
    
    
    #**************** To be completed ****************
    T = np.eye(3,3)
    errors = np.zeros((1,100))
    iterations = 100
    total_time=0
    [T, errors,iterations,total_time] = icp(data, ref, init_pose=None, max_iterations=20, tolerance=0.001)
    print("ICP converged in",i,"iterations, lasted",total_time,"s and the matrix T is equal to\n",T)    
    
    # Draw results
    fig = plt.figure(1, figsize=(20, 5))
    ax = fig.add_subplot(131, projection='3d')
    # Draw reference
    datatools.draw_data(ref, title='Reference', ax=ax)
    
    ax = fig.add_subplot(132, projection='3d')
    # Draw original data and reference
    datatools.draw_data_and_ref(data, ref=ref, title='Raw data', ax=ax)
    
    ##########################################################################
    # Apply transformation found with ICP to data
    #**************** To be completed ****************
    C = data

    ax = fig.add_subplot(133, projection='3d')
    #Draw transformed data and reference
    #**************** To be uncommented and completed ****************
    #datatools.draw_data_and_ref(..., ..., title='Registered data', ax=ax);
    
    plt.show(block=True)
    
    #Display error progress over time
    #**************** To be uncommented and completed ****************
    #fig1 = plt.figure(2, figsize=(20,3))
    #it = np.arange(0,len(errors),1)
    #plt.plot(it, ...)
    #plt.ylabel('Residual distance')
    #plt.xlabel('Iterations')
    #plt.title('Total elapsed time :'+str(...)+' s.')
    #fig1.show()
