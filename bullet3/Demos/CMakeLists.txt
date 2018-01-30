
SUBDIRS( HelloWorld )


IF (USE_GLUT)
	IF (GLUT_FOUND)
		IF(BUILD_CPU_DEMOS)
			IF(BUILD_EXTRAS)
				SUBDIRS( BulletXmlImportDemo ConcaveDemo  ConvexDecompositionDemo SerializeDemo )
			ENDIF()
			SET(SharedDemoSubdirs
				OpenGL   
				Hexapod
			)
		ELSE()
			SET(SharedDemoSubdirs			OpenGL		)
		ENDIF(BUILD_CPU_DEMOS)
		

		SUBDIRS( 		
			${SharedDemoSubdirs}
			Benchmarks
		)
		

	ENDIF(GLUT_FOUND)
ELSE (USE_GLUT)
	IF (WIN32)
		SUBDIRS( 
		OpenGL 
		BasicDemo
		RaytestDemo
		FractureDemo
		Benchmarks 
		Box2dDemo 
		CollisionInterfaceDemo 
		ConcaveDemo 
		ConstraintDemo
		RollingFrictionDemo
		ConvexDecompositionDemo 
		InternalEdgeDemo
		GyroscopicDemo
		FeatherstoneMultiBodyDemo
		GenericJointDemo 
		SerializeDemo 
		SoftDemo
		VoronoiFractureDemo
	)
	ENDIF(WIN32)
ENDIF (USE_GLUT)

