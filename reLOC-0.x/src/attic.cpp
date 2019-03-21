/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.20-kruh                               */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* attic.cpp / 0.20-kruh_048                                                  */
/*----------------------------------------------------------------------------*/
// attic.cpp / 0.12-tokyo_087
/*----------------------------------------------------------------------------*/
	printf("Counting 3\n");
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    ++N_neighbors;
			}
		    }
		    sIndexableBitIdentifier edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "edge_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(MDD[robot_id][layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		    encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u] = edge_occupancy_by_water__;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u]);
		}
	    }
	}
	printf("Counting 4\n");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    typedef std::map<int, int> VerticesMDD_map;
	    typedef std::vector<VerticesMDD_map> LayerVerticesMDD_vector;
	    LayerVerticesMDD_vector layer_Vertices_MDD;
	    layer_Vertices_MDD.resize(N_Layers + 1);

	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
	      for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		  layer_Vertices_MDD[layer][MDD[robot_id][layer][u]] = u;
		}
	    }

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    int neighbor_index = 0;

		    const sVertex *vertex = m_environment.get_Vertex(MDD[robot_id][layer][u]);
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		      {
			VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find((*neighbor)->m_target->m_id);
			if (v_iter != layer_Vertices_MDD[layer + 1].end())
			  {
			    int v = v_iter->second;

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find(vertex->m_id);

		    if (v_iter != layer_Vertices_MDD[layer + 1].end())
		      {
			  int v = v_iter->second;

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
		      }
		       
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    if (mutex_target_Identifiers.size() > 1)
		      {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		      }
		}

		/* Believed to be O.K. */
		if (mutex_vertex_Identifiers.size() > 1)
		  {
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_vertex_Identifiers);
		  }
	    }
	}
    	printf("Counting 5\n");
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    if (mutex_vertex_Identifiers.size() > 1)
	      {
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_vertex_Identifiers);
	      }
	}

	for (int layer = 0; layer <= N_Layers; ++layer)
	  {
	    typedef std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> VertexSpecifiedBitIdentifiers_vector;
	    VertexSpecifiedBitIdentifiers_vector vertex_mutex_occupancy_Identifiers;
	    vertex_mutex_occupancy_Identifiers.resize(N_Vertices);

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	      {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		  {
		    vertex_mutex_occupancy_Identifiers[MDD[robot_id][layer][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		  }
	      }

	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	      {
		if (vertex_mutex_occupancy_Identifiers[vertex_id].size() > 1)
		  {
		    if (vertex_mutex_occupancy_Identifiers[vertex_id].size() > 1)
		      {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 vertex_mutex_occupancy_Identifiers[vertex_id]);
		      }
		  }
	      }
	  }
	printf("Counting 6\n");

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_occupancy_Identifiers);
		}
	    }
	}
	printf("Counting 7\n");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													 sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}








	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    typedef std::map<int, int> VerticesMDD_map;
	    typedef std::vector<VerticesMDD_map> LayerVerticesMDD_vector;
	    LayerVerticesMDD_vector layer_Vertices_MDD;
	    layer_Vertices_MDD.resize(N_Layers + 1);

	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
	      for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		  layer_Vertices_MDD[layer][MDD[robot_id][layer][u]] = u;
		}
	    }

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    int neighbor_index = 0;
		    
		    const sVertex *vertex = m_environment.get_Vertex(MDD[robot_id][layer][u]);
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		      {
			VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find((*neighbor)->m_target->m_id);
			if (v_iter != layer_Vertices_MDD[layer + 1].end())
			  {
			    int v = v_iter->second;

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			  }
		      }
		    {
		    VerticesMDD_map::const_iterator v_iter = layer_Vertices_MDD[layer + 1].find(vertex->m_id);

		    if (v_iter != layer_Vertices_MDD[layer + 1].end())
			{
			  int v = v_iter->second;
			  
			  mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));
			  
			  Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														       sIntegerIndex(neighbor_index)),
											       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														       sIntegerIndex(v)));
			  neighbor_index++;
			}
		    }
		    /*	    
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			  printf("Generating 3.0:%d,%d,%d,%d\n", v, layer, u, robot_id);  
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    printf("Generating 3.1\n");  
		    */
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);
		   if (mutex_target_Identifiers.size() > 1)
		     {
		       Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												   mutex_target_Identifiers);   
		     }
		}
		/* Believed to be O.K. */		
		if (mutex_vertex_Identifiers.size() > 1)
		{
		  Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											      mutex_vertex_Identifiers);
		}
	       
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//s_GlobalPhaseStatistics.enter_Phase("Pregen 3");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    if (mutex_vertex_Identifiers.size() > 1)
	      {
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_vertex_Identifiers);
	      }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//s_GlobalPhaseStatistics.enter_Phase("Pregen 4");

	for (int layer = 0; layer <= N_Layers; ++layer)
	  {
	    typedef std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> VertexSpecifiedBitIdentifiers_vector;
	    VertexSpecifiedBitIdentifiers_vector vertex_mutex_occupancy_Identifiers;
	    vertex_mutex_occupancy_Identifiers.resize(N_Vertices);

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	      {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		  {
		    vertex_mutex_occupancy_Identifiers[MDD[robot_id][layer][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		  }
	      }

	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	      {
		if (vertex_mutex_occupancy_Identifiers[vertex_id].size() > 1)
		  {
		    if (vertex_mutex_occupancy_Identifiers[vertex_id].size() > 1)
		      {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    vertex_mutex_occupancy_Identifiers[vertex_id]);
		      }
		  }
	      }
	  }

	//s_GlobalPhaseStatistics.leave_Phase();
	//s_GlobalPhaseStatistics.enter_Phase("Pregen 5");

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_occupancy_Identifiers);
		}
	    }
	}


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Goaling");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													    sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
