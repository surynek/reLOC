/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.21-robik                              */
/*                                                                            */
/*                  (C) Copyright 2011 - 2021 Pavel Surynek                   */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* simplify_main.cpp / 0.21-robik_054                                         */
/*----------------------------------------------------------------------------*/
//
// Simplify - main program.
//
// CNF formula simplification by clause elimination.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/times.h>
#include <unistd.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"
#include "version.h"

#include "simplify_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("================================================================\n");
	printf("%s : Simplification by Clause Elimination\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("----------------------------------------------------------------\n");
    }


    void print_ConcludingMessage(void)
    {
	printf("----------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("simplify_reLOC  --cnf-file=<string>\n");
	printf("                --out-file=<string>\n");
	printf("                --label=<string>\n");
	printf("\n");
    }


    sResult simplify_Formula(const sCommandParameters &parameters)
    {
	sResult result;
	Formula_list formula;

	FILE *file;
	if ((file = fopen(parameters.m_cnf_filename.c_str(), "r")) == NULL)
	{
	    printf("Error: Cannot open file with cnf formula (code = %d).\n", sSIMPLIFY_PROGRAM_INPUT_OPEN_ERROR);
	    return sSIMPLIFY_PROGRAM_INPUT_OPEN_ERROR;
	}
	if (sFAILED(result = read_Formula(file, formula)))
	{
	    printf("Error: Cannot read a formula (code = %d).\n", result);
	    return result;
	}	
	fclose(file);
	
	Formula_list eliminated_formula;
	if (sFAILED(result = eliminate_Clauses(formula, parameters.m_clause_label, eliminated_formula)))
	{
	    printf("Error: Cannot eliminate clauses from the formula (code = %d).\n", result);
	    return result;
	}

	FILE *out_file;
	if ((out_file = fopen(parameters.m_simplified_cnf_filename.c_str(), "w")) == NULL)
	{
	    printf("Error: Cannot open file to save simplified formula (code = %d).\n", sSIMPLIFY_PROGRAM_OUTPUT_OPEN_ERROR);
	    return sSIMPLIFY_PROGRAM_OUTPUT_OPEN_ERROR;
	}
	write_Formula(out_file, eliminated_formula);
	fclose(out_file);
	
        #ifdef sSTATISTICS
	{
	  s_GlobalPhaseStatistics.to_Screen();
	}
	#endif

	return sRESULT_SUCCESS;
    }


    sResult read_Formula(FILE *file, Formula_list &formula)
    {
	int V, C;
	sString label;

	read_CommentsDIMACS(file, label);
	fscanf(file, "p cnf %d %d\n", &V, &C);
	
	while (!feof(file))
	{
	    read_CommentsDIMACS(file, label); 
	    int l = 0;

	    sClause clause;

	    while (true)
	    {
		fscanf(file, "%d", &l);
	
		if (l == 0)
		{
		    fscanf(file, "\n");
		    break;
		}
		clause.m_clause_label = label;
		clause.m_Literals.push_back(l);
	    }
	    formula.push_back(clause);
	}
	return sRESULT_SUCCESS;
    }


    void write_Formula(FILE *file, const Formula_list &formula)
    {
	sString current_label;

	for (Formula_list::const_iterator clause = formula.begin(); clause != formula.end(); ++clause)
	{
	    if (current_label != clause->m_clause_label)
	    {
		fprintf(file, "c label %s\n", clause->m_clause_label.c_str());
		current_label = clause->m_clause_label.c_str();
	    }
	    for (Clause_vector::const_iterator literal = clause->m_Literals.begin(); literal != clause->m_Literals.end(); ++literal)
	    {
		fprintf(file, "%d ", *literal);
	    }
	    fprintf(file, "0\n");
	}
    }


    sResult eliminate_Clauses(const Formula_list &formula, const sString &label, Formula_list &simplified_formula)
    {
	simplified_formula = formula;

	Formula_list::iterator clause = simplified_formula.begin();
	bool eliminated_all = true;

	sUInt_32 clause_cnt = simplified_formula.size();
	sUInt_32 clause_index = 1;

	while (clause != simplified_formula.end())
	{
	    printf("Progress: %.3f ...\n", 100 * (double)clause_index / clause_cnt);

	    if (label.empty() || clause->m_clause_label.find(label) == 0)
	    {
		FILE *fw;
		sString eliminate_filename = "_simplify/eliminate_" + sInt_32_to_String(getpid()) + ".cnf";
		sString answer_filename = "_simplify/answer_" + sInt_32_to_String(getpid()) + ".txt";
		
		if ((fw = fopen(eliminate_filename.c_str(), "w")) == NULL)
		{
		    return sSIMPLIFY_PROGRAM_ELIMINATE_OPEN_ERROR;
		}
		for (Formula_list::const_iterator other_clause = simplified_formula.begin(); other_clause != simplified_formula.end(); ++other_clause)
		{
		    if (clause != other_clause)
		    {
			for (Clause_vector::const_iterator literal = other_clause->m_Literals.begin(); literal != other_clause->m_Literals.end(); ++literal)
			{		
			    fprintf(fw, "%d ", *literal);
			}
			fprintf(fw, "0\n");		    
		    }
		}
		for (Clause_vector::const_iterator literal = clause->m_Literals.begin(); literal != clause->m_Literals.end(); ++literal)
		{		
		    fprintf(fw, "%d 0\n", -(*literal));
		}
		fclose(fw);
		
		sString system_call = sString("../../sat/glucose3") + " " + eliminate_filename + " " + answer_filename +  " 1>/dev/null 2>/dev/null";
		
		int system_result = system(system_call.c_str());
		if (system_result < 0)
		{
		    return sSIMPLIFY_PROGRAM_SAT_CALL_ERROR;
		}
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
 	        #endif

		FILE *fr;
		if ((fr = fopen(answer_filename.c_str(), "r")) == NULL)
		{
		    return sSIMPLIFY_PROGRAM_ANSWER_OPEN_ERROR;
		}
		
		char answer[32];
		fscanf(fr, "%s\n", answer);
		fclose(fr);
	
		if (strcmp(answer, "SAT") == 0)
		{
		    ++clause;
		    eliminated_all = false;
		}
		else
		{
		    Formula_list::iterator clause_erase = clause++;
		    simplified_formula.erase(clause_erase);
		}
	    }
	    else
	    {
		++clause;
	    }
	    ++clause_index;
	}
	if (eliminated_all)
	{
	    printf("ALL selected clauses eliminated !\n"); 
	}
	else
	{
	    printf("NOT ALL selected clauses eliminated.\n"); 
	}
	return sRESULT_SUCCESS;
    }


    void skip_CommentsDIMACS(FILE *file)
    {
	char ch;
	
	while (true)
	{
	    fscanf(file, "%c", &ch);
	    
	    if (ch == 'c')
	    {
		while (fgetc(file) != '\n');
	    }
	    else
	    {
		ungetc(ch, file);
		return;
	    }
	}
    }


    void read_CommentsDIMACS(FILE *file, sString &clause_label)
    {
	char ch;
	sString label;
	
	while (true)
	{
	    fscanf(file, "%c", &ch);
	    
	    if (ch == 'c')
	    {
		int w;
		sString word;
		bool labeling = false;

		while ((w = fgetc(file)) != '\n')
		{
		    if (labeling)
		    {
			label += w;
		    }
		    else
		    {
			if (w == ' ')
			{
			    if (word == "label")
			    {
				labeling = true;
			    }
			    word.clear();
			}
			else
			{
			    word += w;
			}
		    }
		}
	    }
	    else
	    {
		ungetc(ch, file);

		if (!label.empty())
		{
		    clause_label = label;
		}
		return;
	    }
	}
	sASSERT(false);
    }


    void print_Formula(const Formula_list &formula)
    {
	sString current_label;

	for (Formula_list::const_iterator clause = formula.begin(); clause != formula.end(); ++clause)
	{
	    if (current_label != clause->m_clause_label)
	    {
		printf("c label %s\n", clause->m_clause_label.c_str());
		current_label = clause->m_clause_label.c_str();
	    }
	    for (Clause_vector::const_iterator literal = clause->m_Literals.begin(); literal != clause->m_Literals.end(); ++literal)
	    {
		printf("%d ", *literal);
	    }
	    printf("0\n");
	}
    }


    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	if (parameter.find("--cnf-file=") == 0)
	{
	    command_parameters.m_cnf_filename = parameter.substr(11, parameter.size());
	}
	else if (parameter.find("--out-file=") == 0)
	{
	    command_parameters.m_simplified_cnf_filename = parameter.substr(11, parameter.size());
	}
	else if (parameter.find("--label=") == 0)
	{
	    command_parameters.m_clause_label = parameter.substr(8, parameter.size());
	}
	else
	{
	    return sSIMPLIFY_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	}
	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int argc, char **argv)
{
    sResult result;
    sCommandParameters command_parameters;

    print_IntroductoryMessage();

    if (argc >= 2 && argc <= 4)
    {
	for (int i = 1; i < argc; ++i)
	{
	    result = parse_CommandLineParameter(argv[i], command_parameters);
	    if (sFAILED(result))
	    {
		printf("Error: Cannot parse command line parameters (code = %d).\n", result);
		print_Help();

		return result;
	    }
	}
	result = simplify_Formula(command_parameters);
	if (sFAILED(result))
	{
	    return result;
	}
    }
    else
    {
	print_Help();
    }
    return sRESULT_SUCCESS;
}

