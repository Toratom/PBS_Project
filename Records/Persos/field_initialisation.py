 # nb_of_ellipsoids = 4
    # nb_of_pairs = 6  # (nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # #radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    # #radii_array = np.array([[0.5, 0.1, 0.5], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # #radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # radii_array = np.array([[0.45, 0.05, 0.45], [0.45, 0.05, 0.45],  [0.45, 0.05, 0.45],  [0.45, 0.05, 0.45]])
    # #radii_array = np.array([[0.45, 0.05, 0.45], [0.05, 0.45, 0.45],  [0.45, 0.05, 0.45],  [0.05, 0.45, 0.45]])
    # #ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]]) + np.array([0., 5., 0.])
    # ini_centers = np.array([[0., 0., 0.], [0.5, 0.5, 0.], [0., 1., 0.], [-0.5, 0.5, 0.]]) + np.array([0., 5., 0.])
    # ini_rotation = np.array([[0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.]])
    # theta = np.radians(90.)
    # u = np.array([0., 0., -1.])
    # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    # ini_rotation[1] = q
    # u = np.array([0., 0., 1.])
    # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    # ini_rotation[3] = q
    # # connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0]])
    # connections = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])
    # #connections = np.array([[]])
    # bodies = np.array([0, 0, 0, 0])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # # ini_angular_velocities[3] = np.array([0., 0., 50.])
    # ini_mass = np.array([1., 1., 1., 1.])
    # gravity = np.array([0., -9.8, 0.])

    # Initialization for DEBUGGING -----------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # nb_of_ellipsoids = 8
    # nb_of_pairs = 12#(nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # #radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    # radii_array = np.array([[0.5, 0.1, 0.5], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1],[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    # #radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    # ini_centers = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.],[1.5, 1.5, 0.], [2.5, 0., 0.], [0., 2.5, 0.], [2.5, 2.5, 0.]]) + np.array([0., 5., 0.])
    # ini_rotation = np.array([[0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.],
    #                          [0., 0., 0., 1.]])
    # connections = np.array([[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4]])
    # bodies = np.array([0, 0, 0, 0, 1, 1, 1, 1])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # # ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    # ini_mass = np.array([1., 1., 1., 10.,1., 10., 1., 1.])
    # gravity = np.array([0., -9.8, 0.])
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    # nb_of_ellipsoids = 1
    # nb_of_pairs = 1 #(nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # radii_array = np.array([[0.5, 0.05, 0.5]]) #np.array([[0.5, 0.1, 0.5]])
    # ini_centers = np.array([[0., 0., 0.]]) + np.array([0., 5., 0.])
    # theta = np.radians(90)
    # u = np.array([0., 0., 1.])
    # q = np.array([u[0] * np.sin(theta / 2.), u[1] * np.sin(theta / 2.), u[2] * np.sin(theta / 2.), np.cos(theta /2.)])
    # #q = np.array([0., 0., 0., 1.])
    # # q = q / np.linalg.norm(q)
    # ini_rotation = np.array([q]) #[x, y, z, w]
    # connections = np.array([[]])
    # bodies = np.array([0])
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_velocities[0] = 0. * np.array([1., -1., 0.])
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities[0] = np.array([0., 0., 0.]) # np.array([0., 0., 0.]) # For testing
    # ini_mass = np.array([1.])
    # gravity = np.array([0., -9.8, 0.])




    # Initialization for PRESENTATION --------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------

    #----------- DUCK:
    # with open('Meshes/duck.pkl', 'rb') as inp:
    #     graph = pickle.load(inp)

    # nb_of_ellipsoids = graph["centers"].shape[0]
    # nb_of_pairs = graph["connections"].shape[0] #582
    # print(nb_of_pairs)
    # height = np.array([0,8,0])
    # radii_array = graph["radii"]
    # ini_centers = graph["centers"] + height
    # ini_rotation = graph["rotations"]
    # connections = graph["connections"]
    # bodies = np.array([0 for i in range(nb_of_ellipsoids)])  # To be changed for multiple ducks
    # ini_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities = np.zeros(ini_centers.shape)
    # # ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    # ini_mass = np.array([10 for i in range(nb_of_ellipsoids)]) * 100
    # gravity = np.array([0., -9.8, 0.])

    '''nb_of_ellipsoids = 28
    nb_of_pairs = 72  # (nb_of_ellipsoids * (nb_of_ellipsoids - 1)) / 2
    # radii_array = np.array([[0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2], [0.2, 0.2, 0.2]])
    offset = -0.5
    height = 2
    radii_array = np.array(
        [[0.05, 0.05, 0.05],  # Cube1 vertexes
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.5, 0.5, 0.05],  # front
         [0.5, 0.5, 0.05],  # back
         [0.5, 0.05, 0.5],  # up
         [0.5, 0.05, 0.5],  # down
         [0.05, 0.5, 0.5],  # right
         [0.05, 0.5, 0.5],
         [0.05, 0.05, 0.05],  # Cube2 vertexes
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.05, 0.05, 0.05],
         [0.5, 0.5, 0.05],  # front
         [0.5, 0.5, 0.05],  # back
         [0.5, 0.05, 0.5],  # up
         [0.5, 0.05, 0.5],  # down
         [0.05, 0.5, 0.5],  # right
         [0.05, 0.5, 0.5]
         ])  # left
    # radii_array = np.array([[0.1, 0.5, 0.1], [0.1, 0.5, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])
    ini_centers = np.array(
        [[0., 0., 0.],  #0 CUBE 1
         [1., 0., 0.],  #1
         [0., 1., 0.],  #2
         [1., 1., 0.],  #3
         [0., 0., 1.],  #4
         [1., 0., 1.],  #5
         [0., 1., 1.],  #6
         [1., 1., 1.],  #7
         [0.5, 0.5, 0.],  #front 8
         [0.5, 0.5, 1.],  #back 9
         [0.5, 1., 0.5],  #up 10
         [0.5, 0., 0.5],  #down 11
         [0., 0.5, 0.5],  #right 12
         [1., 0.5, 0.5],  #left 13
         [0. + offset, 0. + height, 0.],  # 0 CUBE 2
         [1. + offset, 0. + height, 0.],  # 1
         [0. + offset, 1. + height, 0.],  # 2
         [1. + offset, 1. + height, 0.],  # 3
         [0. + offset, 0. + height, 1.],  # 4
         [1. + offset, 0. + height, 1.],  # 5
         [0. + offset, 1. + height, 1.],  # 6
         [1. + offset, 1. + height, 1.],  # 7
         [0.5 + offset, 0.5 + height, 0.],  # front 8
         [0.5 + offset, 0.5 + height, 1.],  # back 9
         [0.5 + offset, 1. + height, 0.5],  # up 10
         [0.5 + offset, 0. + height, 0.5],  # down 11
         [0. + offset, 0.5 + height, 0.5],  # right 12
         [1. + offset, 0.5 + height, 0.5]  # left 13
         ]) + np.array([0., 1., 0.])
    ini_rotation = np.array([[0., 0., 0., 1.], # CUBE1
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],  # CUBE2
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.],
                             [0., 0., 0., 1.]
                             ])
    shift = 14
    connections = np.array([[0, 1], [0, 2], [0, 4], [0, 8], [0, 11], [0, 13], # CUBE1
                            [1, 3], [1, 5], [1, 8], [1, 11], [1, 12],
                            [2, 6], [2, 3], [2, 8], [2, 10], [2, 13],
                            [3, 7], [3, 8], [3, 10], [3, 12],
                            [4, 6], [4, 5], [4, 13], [4, 11], [4, 9],
                            [5, 7], [5, 11], [5, 12], [5, 9],
                            [6, 7], [6, 10], [6, 13], [6, 9],
                            [7, 10], [7, 12], [7, 9],
                            [0 + shift, 1 + shift], [0 + shift, 2 + shift], [0 + shift, 4 + shift], [0 + shift, 8 + shift], [0 + shift, 11 + shift], [0 + shift, 13 + shift],  # CUBE2
                            [1 + shift, 3 + shift], [1 + shift, 5 + shift], [1 + shift, 8 + shift], [1 + shift, 11 + shift], [1 + shift, 12 + shift],
                            [2 + shift, 6 + shift], [2 + shift, 3 + shift], [2 + shift, 8 + shift], [2 + shift, 10 + shift], [2 + shift, 13 + shift],
                            [3 + shift, 7 + shift], [3 + shift, 8 + shift], [3 + shift, 10 + shift], [3 + shift, 12 + shift],
                            [4 + shift, 6 + shift], [4 + shift, 5 + shift], [4 + shift, 13 + shift], [4 + shift, 11 + shift], [4 + shift, 9 + shift],
                            [5 + shift, 7 + shift], [5 + shift, 11 + shift], [5 + shift, 12 + shift], [5 + shift, 9 + shift],
                            [6 + shift, 7 + shift], [6 + shift, 10 + shift], [6 + shift, 13 + shift], [6 + shift, 9 + shift],
                            [7 + shift, 10 + shift], [7 + shift, 12 + shift], [7 + shift, 9 + shift]
                            ])
    bodies = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    ini_velocities = np.zeros(ini_centers.shape)
    ini_angular_velocities = np.zeros(ini_centers.shape)
    # ini_angular_velocities[0] = np.array([5., 0., 0.]) # For testing
    ini_mass = np.array([10., 10., 10., 10., 10., 10., 10., 10., 40., 40., 40., 40., 40., 40., 10., 10., 10., 10., 10., 10., 10., 10., 40., 40., 40., 40., 40., 40.]) * 100
    gravity = np.array([0., -9.8, 0.])'''

    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------
    # ----------------------------------------------------------------------------------------------