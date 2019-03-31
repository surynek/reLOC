/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.20-kruh                               */
/*                                                                            */
/*                      (C) Copyright 2019 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* cnf_test.cpp / 0.20-kruh_056                                               */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF production tools - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "cnf.h"

#include "cnf_test.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_indexed_identifier_1(void)
    {
	printf("Indexable identifier test 1 ...\n");

	sVariableStore_CNF cnf_variable_store;
	sIndexableIdentifier alpha(&cnf_variable_store, "Alpha");

	sIndexableIdentifier beta(&cnf_variable_store, "Beta", sIntegerScope(10, 14));

	sIndexableIdentifier gamma(&cnf_variable_store, "Gamma", sIntegerScope(10, 14),
				                               sIntegerScope(10, 14));

	sIndexableIdentifier delta(&cnf_variable_store, "Delta", sIntegerScope(1, 3),
				                               sIntegerScope(10, 14),
				                               sIntegerScope(10, 14));

	alpha.to_Screen();
	beta.to_Screen();
	gamma.to_Screen();
	delta.to_Screen();

	sString string_1 = delta.calc_String(sIntegerIndex(2), sIntegerIndex(10), sIntegerIndex(14));
	printf("Identifier 1: %s\n", string_1.c_str());
	sString string_2 = delta.calc_String(sIntegerIndex(1), sIntegerIndex(11), sIntegerIndex(12));
	printf("Identifier 2: %s\n", string_2.c_str());

	int cnf_index_1 = delta.calc_CNF(sIntegerIndex(2), sIntegerIndex(10), sIntegerIndex(14));
	printf("Identifier 1: %d\n", cnf_index_1);
	int cnf_index_2 = delta.calc_CNF(sIntegerIndex(1), sIntegerIndex(11), sIntegerIndex(12));
	printf("Identifier 2: %d\n", cnf_index_2);

	printf("Indexable identifier test 1 ... finished\n");
    }


    void test_indexed_identifier_2(void)
    {
	printf("Indexable identifier test 2 ...\n");

	sVariableStore_CNF cnf_variable_store;

	sIndexableIdentifier::IndexScopes_vector index_Scopes;
	sIntegerScope scope_1(1, 4);
	sIntegerScope scope_2(1, 4);
	sIntegerScope scope_3(1, 2);
	sIntegerScope scope_4(1, 2);
	index_Scopes.push_back(&scope_1);
	index_Scopes.push_back(&scope_2);
	index_Scopes.push_back(&scope_3);
	index_Scopes.push_back(&scope_4);

	sIndexableIdentifier omega(&cnf_variable_store, "Omega", index_Scopes);
	omega.to_Screen();

	printf("Indexable identifier test 2 ... finished\n");
    }


    void test_indexed_state_identifier_1(void)
    {
	printf("Indexable state identifier test 1 ...\n");

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 5);

	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 4, sIntegerScope(10, 14));

	sIndexableStateIdentifier gamma(&cnf_variable_store, "Gamma", 2, sIntegerScope(10, 14),
				                                       sIntegerScope(10, 14));

	sIndexableStateIdentifier delta(&cnf_variable_store, "Delta", 7, sIntegerScope(1, 3),
				                                       sIntegerScope(10, 14),
				                                       sIntegerScope(10, 14));

	alpha.to_Screen();
	beta.to_Screen();
	gamma.to_Screen();
	delta.to_Screen();

	printf("Indexable state identifier test 1 ... finished\n");
    }


    void test_indexed_state_identifier_2(void)
    {
	printf("Indexable state identifier test 2 ...\n");

	sVariableStore_CNF cnf_variable_store;

	sIndexableStateIdentifier::IndexScopes_vector index_Scopes;
	sIntegerScope scope_1(1, 4);
	sIntegerScope scope_2(1, 4);
	sIntegerScope scope_3(1, 2);
	sIntegerScope scope_4(1, 2);
	index_Scopes.push_back(&scope_1);
	index_Scopes.push_back(&scope_2);
	index_Scopes.push_back(&scope_3);
	index_Scopes.push_back(&scope_4);

	sIndexableStateIdentifier omega(&cnf_variable_store, "Omega", 3, index_Scopes);
	omega.to_Screen();

	printf("Indexable state identifier test 2 ... finished\n");
    }


    void test_clause_generator_1(void)
    {
	printf("Clause generator test 1 ...\n");

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 7, sIntegerScope(1, 8));
	alpha.to_Screen();

	sStateClauseGenerator clause_generator(&cnf_variable_store);

	clause_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&alpha, sIntegerIndex(5)), 3, true);
	clause_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&alpha, sIntegerIndex(5)), 3, false);

	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 227, sIntegerScope(1, 8), sIntegerScope(1, 2));
	beta.to_Screen();

	sIndexableStateIdentifier gamma(&cnf_variable_store, "Gamma", 227, sIntegerScope(1, 4));
	gamma.to_Screen();

	clause_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&beta, sIntegerIndex(4), sIntegerIndex(1)), 19, true);
	clause_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&beta, sIntegerIndex(4), sIntegerIndex(1)), 19, false);

	clause_generator.generate_Equality(stdout,
					   sSpecifiedStateIdentifier(&beta, sIntegerIndex(3), sIntegerIndex(1)),
					   sSpecifiedStateIdentifier(&gamma, sIntegerIndex(4)),
					   true);
	
	clause_generator.generate_Equality(stdout,
					   sSpecifiedStateIdentifier(&beta, sIntegerIndex(3), sIntegerIndex(1)),
					   sSpecifiedStateIdentifier(&gamma, sIntegerIndex(4)),
					   false);

	printf("Clause generator test 1 ... finished\n");
    }


    void test_clause_generator_2(void)
    {
	printf("Clause generator test 2 ...\n");

	int aux_Variable_cnt = 0;

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier chi(&cnf_variable_store, "Chi", 11, sIntegerScope(1, 8));
	chi.to_Screen();

	sIndexableStateIdentifier pi(&cnf_variable_store, "Pi", 11, sIntegerScope(1, 8));
	pi.to_Screen();

	sIndexableStateIdentifier ksi(&cnf_variable_store, "Ksi", 5, sIntegerScope(1, 4), sIntegerScope(1, 4));
	ksi.to_Screen();

	sIndexableStateIdentifier phi(&cnf_variable_store, "Phi", 5, sIntegerScope(1, 4), sIntegerScope(1, 4));
	phi.to_Screen();

	sStateClauseGenerator clause_generator(&cnf_variable_store);
	int total_Literal_cnt = 0;

	printf("|alignment|=%d\n", clause_generator.count_Alignment(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&chi, sIntegerIndex(2))));
	clause_generator.generate_Alignment(stdout, sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)), true);
	clause_generator.generate_Alignment(stdout, sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)), false);

	printf("|conditional|=%d\n", clause_generator.count_ConditionalEquality(aux_Variable_cnt,
										total_Literal_cnt,
										sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)),
										sSpecifiedStateIdentifier(&pi, sIntegerIndex(5)),
										sSpecifiedStateIdentifier(&ksi, sIntegerIndex(2), sIntegerIndex(2)),
										sSpecifiedStateIdentifier(&phi, sIntegerIndex(3), sIntegerIndex(3))));

	clause_generator.generate_ConditionalEquality(stdout,
						      sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&pi, sIntegerIndex(5)),
						      sSpecifiedStateIdentifier(&ksi, sIntegerIndex(2), sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&phi, sIntegerIndex(3), sIntegerIndex(3)),
						      true);

	clause_generator.generate_ConditionalEquality(stdout,
						      sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&pi, sIntegerIndex(5)),
						      sSpecifiedStateIdentifier(&ksi, sIntegerIndex(2), sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&phi, sIntegerIndex(3), sIntegerIndex(3)),
						      false);

	printf("|conditional|=%d\n", clause_generator.count_ConditionalEquality(aux_Variable_cnt,
										total_Literal_cnt,
										sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)),
										sSpecifiedStateIdentifier(&pi, sIntegerIndex(5)),
										sSpecifiedStateIdentifier(&ksi, sIntegerIndex(2), sIntegerIndex(2)),
										sSpecifiedStateIdentifier(&phi, sIntegerIndex(3), sIntegerIndex(3)),
										sSpecifiedStateIdentifier(&ksi, sIntegerIndex(1), sIntegerIndex(1)),
										sSpecifiedStateIdentifier(&phi, sIntegerIndex(2), sIntegerIndex(2))));

	clause_generator.generate_ConditionalEquality(stdout,
						      sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&pi, sIntegerIndex(5)),
						      sSpecifiedStateIdentifier(&ksi, sIntegerIndex(2), sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&phi, sIntegerIndex(3), sIntegerIndex(3)),
						      sSpecifiedStateIdentifier(&ksi, sIntegerIndex(1), sIntegerIndex(1)),
						      sSpecifiedStateIdentifier(&phi, sIntegerIndex(2), sIntegerIndex(2)),
						      true);

	clause_generator.generate_ConditionalEquality(stdout,
						      sSpecifiedStateIdentifier(&chi, sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&pi, sIntegerIndex(5)),
						      sSpecifiedStateIdentifier(&ksi, sIntegerIndex(2), sIntegerIndex(2)),
						      sSpecifiedStateIdentifier(&phi, sIntegerIndex(3), sIntegerIndex(3)),
						      sSpecifiedStateIdentifier(&ksi, sIntegerIndex(1), sIntegerIndex(1)),
						      sSpecifiedStateIdentifier(&phi, sIntegerIndex(2), sIntegerIndex(2)),
						      false);

	printf("Clause generator test 2 ... finished\n");
    }


    void test_difference_generator_1(void)
    {
	printf("Difference generator test 1 ...\n");

	int aux_Variable_cnt = 0;

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 125, sIntegerScope(1, 8));
	alpha.to_Screen();

	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 125, sIntegerScope(1, 8));
	beta.to_Screen();

	sIndexableStateIdentifier gamma(&cnf_variable_store, "Gamma", 125, sIntegerScope(1, 8));
	gamma.to_Screen();

	sIndexableStateIdentifier delta(&cnf_variable_store, "Delta", 125, sIntegerScope(1, 8));
	delta.to_Screen();

	sStateClauseGenerator::SpecifiedStateIdentifiers_vector diff_state_Identifiers;
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(4)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(1)));

	sStateClauseGenerator clause_generator(&cnf_variable_store);
	int total_Literal_cnt = 0;

	printf("|difference|=%d\n", clause_generator.count_DifferenceConstraint(aux_Variable_cnt,
										total_Literal_cnt,
										sSpecifiedStateIdentifier(&alpha, sIntegerIndex(1)),
										diff_state_Identifiers));
	printf("number of auxiliary variables=%d\n", aux_Variable_cnt);

	clause_generator.generate_DifferenceConstraint(stdout,
						       sSpecifiedStateIdentifier(&alpha, sIntegerIndex(1)),
						       diff_state_Identifiers,
						       true);

	printf("|all difference|=%d\n", clause_generator.count_AllDifferenceConstraint(aux_Variable_cnt,
										       total_Literal_cnt,
										       diff_state_Identifiers));
	printf("number of auxiliary variables=%d\n", aux_Variable_cnt);

	clause_generator.generate_AllDifferenceConstraint(stdout,
							  diff_state_Identifiers,
							  true);

	clause_generator.generate_AllDifferenceConstraint(stdout,
							  diff_state_Identifiers,
							  false);
	
	printf("Difference generator test 1 ... finished\n");
    }


    void test_case_split_generator_1(void)
    {
	printf("Case split generator test 1 ...\n");

	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 11, sIntegerScope(1, 8));
	alpha.to_Screen();

	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 11, sIntegerScope(1, 8));
	beta.to_Screen();

	sIndexableStateIdentifier gamma(&cnf_variable_store, "Gamma", 11, sIntegerScope(1, 8));
	gamma.to_Screen();

	sIndexableStateIdentifier delta(&cnf_variable_store, "Delta", 11, sIntegerScope(1, 8));
	delta.to_Screen();

	sStateClauseGenerator::SpecifiedStateIdentifiers_vector case_split_Identifiers;
	case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(2)));
	case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(3)));
	case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(4)));
	case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(1)));

	sStateClauseGenerator::States_vector case_split_States;
	case_split_States.push_back(7);
	case_split_States.push_back(8);
	case_split_States.push_back(9);
	case_split_States.push_back(10);

	sStateClauseGenerator clause_generator(&cnf_variable_store);

	printf("|case split|=%d\n", clause_generator.count_CaseSplitting(aux_Variable_cnt,
									 total_Literal_cnt,
									 case_split_Identifiers,
									 case_split_States));
	printf("number of auxiliary variables=%d\n", aux_Variable_cnt);
	
	clause_generator.generate_CaseSplitting(stdout,
						case_split_Identifiers,
						case_split_States,
						true);	
       
	printf("Case split generator test 1 ... finished\n");
    }


    void test_LEX_less_generator_1(void)
    {
	printf("LEX less generator test 1 ...\n");

	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 11, sIntegerScope(1, 8));
	alpha.to_Screen();

	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 11, sIntegerScope(1, 8));
	beta.to_Screen();

	sAdvancedClauseGenerator advanced_generator(&cnf_variable_store);

	printf("|LEX less|=%d\n", advanced_generator.count_LEXLess_Constraint(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&alpha, sIntegerIndex(2)),
									      sSpecifiedStateIdentifier(&beta, sIntegerIndex(3))));
	printf("number of auxiliary variables=%d\n", aux_Variable_cnt);
	       
	advanced_generator.generate_LEXLess_Constraint(stdout,
						       sSpecifiedStateIdentifier(&alpha, sIntegerIndex(2)),
						       sSpecifiedStateIdentifier(&beta, sIntegerIndex(3)),
						       true);

	advanced_generator.generate_LEXLess_Constraint(stdout,
						       sSpecifiedStateIdentifier(&alpha, sIntegerIndex(2)),
						       sSpecifiedStateIdentifier(&beta, sIntegerIndex(3)),
						       false);

	advanced_generator.generate_LEXLess_Constraint(stdout,
						       sSpecifiedStateIdentifier(&beta, sIntegerIndex(3)),
						       sSpecifiedStateIdentifier(&alpha, sIntegerIndex(2)),
						       false);
       
	printf("LEX less generator test 1 ... finished\n");
    }


    void test_difference_generator_2(void)
    {
	printf("c Difference generator test 2 ...\n");

	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int Clause_cnt = 0;

	sVariableStore_CNF cnf_variable_store, cnf_variable_store_2;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 8, sIntegerScope(1, 8));
	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 8, sIntegerScope(1, 8));
	sIndexableStateIdentifier gamma(&cnf_variable_store, "Gamma", 8, sIntegerScope(1, 8));
	sIndexableStateIdentifier delta(&cnf_variable_store, "Delta", 8, sIntegerScope(1, 8));

	sStateClauseGenerator::SpecifiedStateIdentifiers_vector diff_state_Identifiers;
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(4)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(1)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(4)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(5)));

	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(5)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(6)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(7)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(1)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(4)));

	sStateClauseGenerator clause_generator(&cnf_variable_store);

	Clause_cnt = clause_generator.count_AllDifferenceConstraint(aux_Variable_cnt,
								    total_Literal_cnt,
								    diff_state_Identifiers);

	printf("p cnf %d %d\n", cnf_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1, Clause_cnt);

	clause_generator.generate_AllDifferenceConstraint(stdout,
							  diff_state_Identifiers,
							  false);

	printf("c Standard all-different encoding:\n");
	printf("c   |all difference|=%d\n", Clause_cnt);
	printf("c   number of auxiliary variables=%d\n", aux_Variable_cnt);

	printf("c Difference generator test 2 ... finished\n");
    }


    void test_difference_generator_3(void)
    {
	printf("c Difference generator test 3 ...\n");

	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int Clause_cnt = 0;

	sVariableStore_CNF cnf_variable_store, cnf_variable_store_2;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", 8, sIntegerScope(1, 8));
	sIndexableStateIdentifier beta(&cnf_variable_store, "Beta", 8, sIntegerScope(1, 8));
	sIndexableStateIdentifier gamma(&cnf_variable_store, "Gamma", 8, sIntegerScope(1, 8));
	sIndexableStateIdentifier delta(&cnf_variable_store, "Delta", 8, sIntegerScope(1, 8));

	sStateClauseGenerator::SpecifiedStateIdentifiers_vector diff_state_Identifiers;
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(4)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(1)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(4)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(5)));

	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(5)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(6)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&beta, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(2)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(3)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&gamma, sIntegerIndex(7)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(1)));
	diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&delta, sIntegerIndex(4)));

	sAdvancedClauseGenerator advanced_generator(&cnf_variable_store);

	aux_Variable_cnt = 0;

	Clause_cnt = advanced_generator.count_AllDifferenceConstraint(aux_Variable_cnt,
								      total_Literal_cnt,
								      diff_state_Identifiers);

	printf("p cnf %d %d\n", cnf_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1, Clause_cnt);

	advanced_generator.generate_AllDifferenceConstraint(stdout,
							    diff_state_Identifiers,
							    false);

	printf("c Bijection-based all-different encoding:\n");
	printf("c   |all difference|=%d\n", Clause_cnt);
	printf("c   number of auxiliary variables=%d\n", aux_Variable_cnt);
	
	printf("c Difference generator test 2 ... finished\n");
    }


    void test_binary_tree_1(void)
    {
	sBinaryTree binary_tree;
	sBinaryTree::Bits_list Bits_1, Bits_2;

    	Bits_1.push_back(true);
	Bits_1.push_back(true);
	Bits_1.push_back(true);
	Bits_1.push_back(true);

    	Bits_2.push_back(true);
	Bits_2.push_back(true);
	Bits_2.push_back(true);
	Bits_2.push_back(false);

	binary_tree.insert(Bits_1);
	binary_tree.to_Screen();
	binary_tree.insert(Bits_2);
	binary_tree.to_Screen();
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    
    test_indexed_identifier_1();
    test_indexed_identifier_2();

    test_indexed_state_identifier_1();
    test_indexed_state_identifier_2();

    test_clause_generator_1();
    test_clause_generator_2();

    test_difference_generator_1();

    test_case_split_generator_1();

    test_LEX_less_generator_1();
    
    test_difference_generator_2();
    test_difference_generator_3();

    test_binary_tree_1();
}

