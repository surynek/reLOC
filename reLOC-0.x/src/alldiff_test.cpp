/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.22-robik                              */
/*                                                                            */
/*                  (C) Copyright 2011 - 2021 Pavel Surynek                   */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* alldiff_test.cpp / 0.22-robik_103                                          */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF production tools - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "cnf.h"

#include "alldiff_test.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_all_different_Standard(int N_Variables, int N_States, int range_size)
    {
	sASSERT(range_size <= N_States);

	int aux_Variable_cnt = 0;
	int Clause_cnt = 0;
	int total_Literal_cnt = 0;

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", N_States, sIntegerScope(1, N_Variables));

	sIndexableStateIdentifier lower(&cnf_variable_store, "lower", N_States, sIntegerScope(1, N_Variables));
	sIndexableStateIdentifier upper(&cnf_variable_store, "upper", N_States, sIntegerScope(1, N_Variables));

	sStateClauseGenerator clause_generator(&cnf_variable_store);

	sStateClauseGenerator::SpecifiedStateIdentifiers_vector diff_state_Identifiers;
	for (int i = 1; i <= N_Variables; ++i)
	{
	    diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));

	    Clause_cnt += clause_generator.count_Alignment(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));
	}

	Clause_cnt += clause_generator.count_AllDifferenceConstraint(aux_Variable_cnt,
								     total_Literal_cnt,
								     diff_state_Identifiers);

	std::vector<std::pair<int, int> > Bounds;

	if (range_size > 0)
	{
	    printf("c Variable ranges\n");

	    for (int i = 1; i <= N_Variables; ++i)
	    {
		int shift_range = N_States - range_size;
		int low = random() % shift_range;
		int up = low + range_size;

		printf("c <%d, %d>\n", low, up);

		Clause_cnt += clause_generator.count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)), low);
		Clause_cnt += clause_generator.count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)), up);

		Bounds.push_back(std::pair<int, int>(low, up));

		Clause_cnt += clause_generator.count_LEXLess_Constraint(aux_Variable_cnt,
									total_Literal_cnt,
									sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)),
									sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));

		Clause_cnt += clause_generator.count_LEXLess_Constraint(aux_Variable_cnt,
									total_Literal_cnt,
									sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)),
									sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)));
	    }
	}

	printf("c Standard all-different encoding:\n");
	printf("c   |all difference|=%d\n", Clause_cnt);
	printf("c   number of auxiliary variables=%d\n", aux_Variable_cnt);
	printf("c   number of literals=%d\n", total_Literal_cnt);

	printf("p cnf %d %d\n", cnf_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1, Clause_cnt);

	clause_generator.generate_AllDifferenceConstraint(stdout,
							  diff_state_Identifiers,
							  false);

	for (int i = 1; i <= N_Variables; ++i)
	{
	    clause_generator.generate_Alignment(stdout, sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));

	    if (range_size > 0)
	    {
		clause_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)), Bounds[i-1].first);
		clause_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)), Bounds[i-1].second);

		clause_generator.generate_LEXLess_Constraint(stdout,
							     sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)),
							     sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));

		clause_generator.generate_LEXLess_Constraint(stdout,
							     sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)),
							     sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)));
	    }
	}
    }


    void test_all_different_Bijection(int N_Variables, int N_States, int range_size)
    {
	sASSERT(range_size <= N_States);

	int aux_Variable_cnt = 0;
	int Clause_cnt = 0;
	int total_Literal_cnt = 0;

	sVariableStore_CNF cnf_variable_store;
	sIndexableStateIdentifier alpha(&cnf_variable_store, "Alpha", N_States, sIntegerScope(1, N_Variables));

	sIndexableStateIdentifier lower(&cnf_variable_store, "lower", N_States, sIntegerScope(1, N_Variables));
	sIndexableStateIdentifier upper(&cnf_variable_store, "upper", N_States, sIntegerScope(1, N_Variables));

	sAdvancedClauseGenerator advanced_generator(&cnf_variable_store);

	sStateClauseGenerator::SpecifiedStateIdentifiers_vector diff_state_Identifiers;
	for (int i = 1; i <= N_Variables; ++i)
	{
	    diff_state_Identifiers.push_back(sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));

	    Clause_cnt += advanced_generator.count_Alignment(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));
	}

	Clause_cnt += advanced_generator.count_AllDifferenceConstraint(aux_Variable_cnt,
								       total_Literal_cnt,
								       diff_state_Identifiers);

	std::vector<std::pair<int, int> > Bounds;

	if (range_size > 0)
	{
	    printf("c Variable ranges\n");

	    for (int i = 1; i <= N_Variables; ++i)
	    {
		int shift_range = N_States - range_size;
		int low = random() % shift_range;
		int up = low + range_size;

		printf("c <%d, %d>\n", low, up);

		Clause_cnt += advanced_generator.count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)), low);
		Clause_cnt += advanced_generator.count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)), up);

		Bounds.push_back(std::pair<int, int>(low, up));
	    
		Clause_cnt += advanced_generator.count_LEXLess_Constraint(aux_Variable_cnt,
									  total_Literal_cnt,
									  sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)),
									  sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));
		
		Clause_cnt += advanced_generator.count_LEXLess_Constraint(aux_Variable_cnt,
									  total_Literal_cnt,
									  sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)),
									  sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)));
	    }
	}

	printf("c Bijection-based all-different encoding:\n");
	printf("c   |all difference|=%d\n", Clause_cnt);
	printf("c   number of auxiliary variables=%d\n", aux_Variable_cnt);
	printf("c   number of literals=%d\n", total_Literal_cnt);

	printf("p cnf %d %d\n", cnf_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1, Clause_cnt);

	advanced_generator.generate_AllDifferenceConstraint(stdout,
							    diff_state_Identifiers,
							    false);

	for (int i = 1; i <= N_Variables; ++i)
	{
	    advanced_generator.generate_Alignment(stdout, sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));

	    if (range_size > 0)
	    {
		advanced_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)), Bounds[i-1].first);
		advanced_generator.generate_Equality(stdout, sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)), Bounds[i-1].second);
	    
		advanced_generator.generate_LEXLess_Constraint(stdout,
							       sSpecifiedStateIdentifier(&lower, sIntegerIndex(i)),
							       sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)));
	    
		advanced_generator.generate_LEXLess_Constraint(stdout,
							       sSpecifiedStateIdentifier(&alpha, sIntegerIndex(i)),
							       sSpecifiedStateIdentifier(&upper, sIntegerIndex(i)));
	    }
	}
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int argc, char **argv)
{
    if (argc != 5)
    {
	printf("Usage:\n");
	printf("$ ./test_alldiff_reLOC <number-of-variables> <number-of-states> <range> [standard|bijection]\n");
    }
    else
    {
	int N_Variables = sInt_32_from_String(argv[1]);
	int N_States = sInt_32_from_String(argv[2]);
	int range = sInt_32_from_String(argv[3]);

	if (strcmp(argv[4], "standard") == 0)
	{
	    test_all_different_Standard(N_Variables, N_States, range);
	}
	else if (strcmp(argv[4], "bijection") == 0)
	{
	    test_all_different_Bijection(N_Variables, N_States, range);
	}
	else
	{
	    sASSERT(false);
	}
    }
}

