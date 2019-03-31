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
/* cnf.cpp / 0.20-kruh_055                                                    */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF formula production tools.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "cnf.h"
#include "statistics.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sIndex

    bool sIndex::operator>=(const sIndex &index) const
    {
	return (index <= *this);
    }


    bool sIndex::operator>(const sIndex &index) const
    {
	return (index < *this);
    }


/*----------------------------------------------------------------------------*/
// sIntegerIndex

    sIntegerIndex::sIntegerIndex(int value)
	: m_value(value)
    {
	// nothing
    }


    sIntegerIndex::sIntegerIndex(const sIntegerIndex &integer_index)
	: m_value(integer_index.m_value)
    {
	// nothing
    }


    const sIntegerIndex& sIntegerIndex::operator=(const sIntegerIndex &integer_index)
    {
	m_value = integer_index.m_value;

	return *this;
    }


    sIndex* sIntegerIndex::clone(void) const
    {
	sIndex *index = new sIntegerIndex(*this);
	return index;
    }


    int sIntegerIndex::get_Value(void) const
    {
	return m_value;
    }


    void sIntegerIndex::set_Value(int value)
    {
	m_value = value;
    }


    void sIntegerIndex::increment(void)
    {
	++m_value;
    }


    void sIntegerIndex::decrement(void)
    {
	--m_value;
    }


    void sIntegerIndex::increment(int distance)
    {
	m_value += distance;
    }


    void sIntegerIndex::decrement(int distance)
    {
	m_value -= distance;
    }


    bool sIntegerIndex::operator==(const sIndex &index) const
    {
	return (m_value == ((sIntegerIndex&)index).m_value);
    }


    bool sIntegerIndex::operator!=(const sIndex &index) const
    {
	return (m_value != ((sIntegerIndex&)index).m_value);
    }


    bool sIntegerIndex::operator<=(const sIndex &index) const
    {
	return (m_value <= ((sIntegerIndex&)index).m_value);
    }


    bool sIntegerIndex::operator<(const sIndex &index) const
    {
	return (m_value < ((sIntegerIndex&)index).m_value);
    }


    int sIntegerIndex::operator-(const sIndex &index) const
    {
	return (m_value - ((sIntegerIndex&)index).m_value);
    }


    sString sIntegerIndex::to_String(void) const
    {
	return sUInt_32_to_String(m_value);
    }


    void sIntegerIndex::to_Screen(void) const
    {
	printf("Integer index (value = %d)\n", m_value);
    }


/*----------------------------------------------------------------------------*/
// sIndex_iterator

    sIndex_iterator::sIndex_iterator(const sIndex &index)
    {
	m_index = index.clone();
    }


    sIndex_iterator::sIndex_iterator(const sIndex_iterator &index_iterator)
    {
	m_index = index_iterator.m_index->clone();
    }


    const sIndex_iterator& sIndex_iterator::operator=(const sIndex_iterator &index_iterator)
    {
	delete m_index;
	m_index = index_iterator.m_index->clone();
	return *this;
    }


    sIndex_iterator::~sIndex_iterator()
    {
	delete m_index;
    }


    sIndex_iterator& sIndex_iterator::operator++()
    {
	m_index->increment();
	return *this;
    }


    sIndex_iterator& sIndex_iterator::operator--()
    {
	m_index->decrement();
	return *this;
    }


    sIndex_iterator& sIndex_iterator::operator+=(int distance)
    {
	m_index->increment(distance);
	return *this;
    }


    sIndex_iterator& sIndex_iterator::operator-=(int distance)
    {
	m_index->decrement(distance);
	return *this;
    }
    

    bool sIndex_iterator::operator==(const sIndex_iterator &iterator) const
    {
	return (*m_index == *iterator);
    }


    bool sIndex_iterator::operator!=(const sIndex_iterator &iterator) const
    {
	return (*m_index != *iterator);
    }


    const sIndex& sIndex_iterator::operator*() const
    {
	return *m_index;
    }


/*----------------------------------------------------------------------------*/
// sIndexScope

    sIndexScope::sIndexScope()
    {
	// nothing
    }


    sIndexScope::sIndexScope(const sIndexScope &sUNUSED(scope))
    {
	// nothing
    }


    const sIndexScope& sIndexScope::operator=(const sIndexScope &sUNUSED(scope))
    {
	return *this;
    }
    

/*----------------------------------------------------------------------------*/
// sIntegerScope

    sIntegerScope::sIntegerScope()
	: m_min(0)
	, m_max(0)
    {
	// nothing
    }

    sIntegerScope::sIntegerScope(int min, int max)
	: m_min(min)
	, m_max(max)
    {
	// nothing
    }
    
    
    sIntegerScope::sIntegerScope(const sIntegerScope &integer_scope)
	: sIndexScope(integer_scope)
	, m_min(integer_scope.m_min)
	, m_max(integer_scope.m_max)
    {
	// nothing
    }

    
    const sIntegerScope& sIntegerScope::operator=(const sIntegerScope &integer_scope)
    {
	sIndexScope::operator=(integer_scope);
	m_min = integer_scope.m_min;
	m_max = integer_scope.m_max;

	return *this;
    }


    int sIntegerScope::get_Size(void) const
    {
	return m_max - m_min + 1;
    }

    
    sIndexScope* sIntegerScope::clone(void) const
    {
	sIndexScope *index_scope = new sIntegerScope(*this);
	return index_scope;
    }


    sIndex_iterator sIntegerScope::get_Begin(void) const
    {
	return sIndex_iterator(sIntegerIndex(m_min));
    }


    sIndex_iterator sIntegerScope::get_End(void) const
    {
	return sIndex_iterator(sIntegerIndex(m_max + 1));
    }


/*----------------------------------------------------------------------------*/
// sVariableStore_CNF

    sVariableStore_CNF::sVariableStore_CNF()
	: m_last_variable(1)
    {
	// nothing
    }


    sVariableStore_CNF::sVariableStore_CNF(const sVariableStore_CNF &variable_store)
	: m_last_variable(variable_store.m_last_variable)
    {
	// nothing
    }


    const sVariableStore_CNF& sVariableStore_CNF::operator=(const sVariableStore_CNF &variable_store)
    {
	m_last_variable = variable_store.m_last_variable;

	return *this;
    }

    int sVariableStore_CNF::get_Last_CNFVariable(void) const
    {
	return m_last_variable;
    }


    void sVariableStore_CNF::alloc_CNFVariables(int N_Variables)
    {
	m_last_variable += N_Variables;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Variables += N_Variables;
	}
	#endif
    }




/*----------------------------------------------------------------------------*/
// sBinaryTreeNode

    sBinaryTreeNode::sBinaryTreeNode()
	: m_is_full(false)
	, m_prev_node(NULL)
	, m_left_node(NULL)
	, m_right_node(NULL)
    {
	// nothing
    }


    sBinaryTreeNode::~sBinaryTreeNode()
    {
	if (m_left_node)
	{
	    delete m_left_node;
	}
	if (m_right_node)
	{
	    delete m_right_node;
	}
    }


    void sBinaryTreeNode::to_Screen(const sString &indent) const
    {
	if (m_left_node != NULL)
	{
	    m_left_node->to_Screen(indent + " ");
	}
	else
	{
	    printf("%s |\n", indent.c_str());
	}
	printf("%s%s\n", indent.c_str(), m_is_full ? "c" : "i");

	if (m_right_node != NULL)
	{
	    m_right_node->to_Screen(indent + " ");
	}
	else
	{
	    printf("%s |\n", indent.c_str());
	}
    }




/*----------------------------------------------------------------------------*/
// sBinaryTree

    sBinaryTree::sBinaryTree()
    {	    
	m_root_node = new sBinaryTreeNode();
    }


    sBinaryTree::~sBinaryTree()
    {
	if (m_root_node)
	{
	    delete m_root_node;
	}
    }


    int sBinaryTree::count_TreeDisequalities(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const
    {
	Bits_list prefix;
	return count_TreeDisequalities(m_root_node, prefix, aux_Variable_cnt, total_Literal_cnt, spec_identifier);
    }


    int sBinaryTree::generate_TreeDisequalities(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string, int weight) const
    {
	int Clause_cnt = 0;
	Bits_list prefix;

	Clause_cnt =  generate_TreeDisequalities(m_root_node, prefix, fw, spec_identifier, string, weight);

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
	return Clause_cnt;
    }


    void sBinaryTree::cast_TreeDisequalities(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int weight)
    {
	Bits_list prefix;

	cast_TreeDisequalities(m_root_node, prefix, solver, spec_identifier, weight);
    }    


    int sBinaryTree::count_TreeDisequalities(const sBinaryTreeNode *tree_node, const Bits_list &prefix, int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const
    {
	int Clause_cnt = 0;

	if (tree_node->m_is_full)
	{
	    ++Clause_cnt;
	}
	else
	{
	    if (tree_node->m_left_node != NULL)
	    {
		Bits_list negative_prefix(prefix);
		negative_prefix.push_back(false);
		
		total_Literal_cnt += prefix.size();
		Clause_cnt += count_TreeDisequalities(tree_node->m_left_node, negative_prefix, aux_Variable_cnt, total_Literal_cnt, spec_identifier);
	    }
	    if (tree_node->m_right_node != NULL)
	    {
		Bits_list positive_prefix(prefix);
		positive_prefix.push_back(true);

		total_Literal_cnt += prefix.size();
		Clause_cnt += count_TreeDisequalities(tree_node->m_right_node, positive_prefix, aux_Variable_cnt, total_Literal_cnt, spec_identifier);
	    }
	}
	return Clause_cnt;
    }


    int sBinaryTree::generate_TreeDisequalities(const sBinaryTreeNode *tree_node, const Bits_list &prefix, FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string, int weight) const
    {
	int Clause_cnt = 0;

	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	if (tree_node->m_is_full)
	{
	    int bit = N_Bits - 1;
	    sASSERT(!prefix.empty());

	    for (Bits_list::const_iterator prefix_bit = prefix.begin(); prefix_bit != prefix.end(); ++prefix_bit, --bit)
	    {
		if (*prefix_bit)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier.calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier.calc_CNF(bit));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier.calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier.calc_CNF(bit));
		    }
		}
	    }
	    ++Clause_cnt;
	    fprintf(fw, " 0\n");
	}
	else
	{
	    if (tree_node->m_left_node != NULL)
	    {
		Bits_list negative_prefix(prefix);
		negative_prefix.push_back(false);
		
		Clause_cnt += generate_TreeDisequalities(tree_node->m_left_node, negative_prefix, fw, spec_identifier, string, weight);
	    }
	    if (tree_node->m_right_node != NULL)
	    {
		Bits_list positive_prefix(prefix);
		positive_prefix.push_back(true);

		Clause_cnt += generate_TreeDisequalities(tree_node->m_right_node, positive_prefix, fw, spec_identifier, string, weight);
	    }
	}
	return Clause_cnt;
    }


    void sBinaryTree::cast_TreeDisequalities(const sBinaryTreeNode *tree_node, const Bits_list &prefix, Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int weight)
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	if (tree_node->m_is_full)
	{
	    int bit = N_Bits - 1;
	    sASSERT(!prefix.empty());
	    
	    std::vector<int> Literals;

	    for (Bits_list::const_iterator prefix_bit = prefix.begin(); prefix_bit != prefix.end(); ++prefix_bit, --bit)
	    {
		if (*prefix_bit)
		{
		    Literals.push_back(-spec_identifier.calc_CNF(bit));
		}
		else
		{
		    Literals.push_back(spec_identifier.calc_CNF(bit));
		}
	    }
	    cast_Clause(solver, Literals);
	}
	else
	{
	    if (tree_node->m_left_node != NULL)
	    {
		Bits_list negative_prefix(prefix);
		negative_prefix.push_back(false);
		
		cast_TreeDisequalities(tree_node->m_left_node, negative_prefix, solver, spec_identifier, weight);
	    }
	    if (tree_node->m_right_node != NULL)
	    {
		Bits_list positive_prefix(prefix);
		positive_prefix.push_back(true);

		cast_TreeDisequalities(tree_node->m_right_node, positive_prefix, solver, spec_identifier, weight);
	    }
	}
    }    


    void sBinaryTree::insert(const Bits_list &Bits)
    {
	sBinaryTreeNode *prev_node;
	sBinaryTreeNode **tree_node = &m_root_node;

	for (Bits_list::const_iterator bit = Bits.begin(); bit != Bits.end(); ++bit)
	{
	    prev_node = *tree_node;
	    if (*bit)
	    {
		tree_node = &(*tree_node)->m_right_node;
	    }
	    else
	    {
		tree_node = &(*tree_node)->m_left_node;
	    }
	    if (*tree_node == NULL)
	    {
		*tree_node = new sBinaryTreeNode();
		(*tree_node)->m_prev_node = prev_node;
		prev_node = *tree_node;
	    }
	}
	sBinaryTreeNode *up_node = *tree_node;
	up_node->m_is_full = true;

	up_node = up_node->m_prev_node;

	while (up_node != NULL)
	{
	    if (up_node->m_left_node != NULL && up_node->m_right_node != NULL)
	    {
		if (up_node->m_left_node->m_is_full && up_node->m_right_node->m_is_full)
		{
		    up_node->m_is_full = true;
		}
		else
		{
		    break;
		}
	    }
	    up_node = up_node->m_prev_node;
	}
    }

/*----------------------------------------------------------------------------*/
    
    void sBinaryTree::cast_Clause(Glucose::Solver *solver, int lit_1)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	
	cast_Clause(solver, Lits);
    }

    
    void sBinaryTree::cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	
	cast_Clause(solver, Lits);
    }

    
    void sBinaryTree::cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2, int lit_3)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	Lits.push_back(lit_3);
		
	cast_Clause(solver, Lits);
    }

    
    void sBinaryTree::cast_Clause(Glucose::Solver *solver, vector<int> &Lits)
    {
	Glucose::vec<Glucose::Lit> glu_Lits;
	
	for (vector<int>::const_iterator lit = Lits.begin(); lit != Lits.end(); ++lit)
	{
	    int glu_var = sABS(*lit) - 1;
	    while (glu_var >= solver->nVars())
	    {
		solver->newVar();
	    }
	    glu_Lits.push((*lit > 0) ? Glucose::mkLit(glu_var) : ~Glucose::mkLit(glu_var));
	}
	solver->addClause(glu_Lits);
    }
    

    void sBinaryTree::to_Screen(const sString &indent) const
    {
	if (m_root_node != NULL)
	{
	    m_root_node->to_Screen(indent);
	}
    }




/*----------------------------------------------------------------------------*/
// sSpecifiedIdentifier

    sSpecifiedIdentifier::sSpecifiedIdentifier()
	: m_indexable_identifier(NULL)
    {
    }


    sSpecifiedIdentifier::sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier)
	: m_indexable_identifier(indexable_identifier)
    {
	// nothing
    }


    sSpecifiedIdentifier::sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const sIndex &index)
	: m_indexable_identifier(indexable_identifier)
    {
	m_scope_Indexes.push_back(index.clone());
    }


    sSpecifiedIdentifier::sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const sIndex &index_1, const sIndex &index_2)
	: m_indexable_identifier(indexable_identifier)
    {
	m_scope_Indexes.push_back(index_1.clone());
	m_scope_Indexes.push_back(index_2.clone());
    }


    sSpecifiedIdentifier::sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const sIndex &index_1, const sIndex &index_2, const sIndex &index_3)
	: m_indexable_identifier(indexable_identifier)
    {
	m_scope_Indexes.push_back(index_1.clone());
	m_scope_Indexes.push_back(index_2.clone());
	m_scope_Indexes.push_back(index_3.clone());
    }


    sSpecifiedIdentifier::sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const Indexes_vector &scope_Indexes)
	: m_indexable_identifier(indexable_identifier)
    {
	for (Indexes_vector::const_iterator index = scope_Indexes.begin(); index != scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}
    }
    

    sSpecifiedIdentifier::sSpecifiedIdentifier(const sSpecifiedIdentifier &specified_identifier)
	: m_indexable_identifier(specified_identifier.m_indexable_identifier)
    {
	for (Indexes_vector::const_iterator index = specified_identifier.m_scope_Indexes.begin(); index != specified_identifier.m_scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}
    }


    const sSpecifiedIdentifier& sSpecifiedIdentifier::operator=(const sSpecifiedIdentifier &specified_identifier)
    {
	m_indexable_identifier = specified_identifier.m_indexable_identifier;

	for (Indexes_vector::const_iterator index = m_scope_Indexes.begin(); index != m_scope_Indexes.end(); ++index)
	{
	    delete *index;
	}

	m_scope_Indexes.clear();

	for (Indexes_vector::const_iterator index = specified_identifier.m_scope_Indexes.begin(); index != specified_identifier.m_scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}

	return *this;
    }


    sSpecifiedIdentifier::~sSpecifiedIdentifier()
    {
	for (Indexes_vector::const_iterator index = m_scope_Indexes.begin(); index != m_scope_Indexes.end(); ++index)
	{
	    delete *index;
	}
    }
    

    sString sSpecifiedIdentifier::calc_String(void) const
    {
	switch(m_scope_Indexes.size())
	{
	case 0:
	{
	    return m_indexable_identifier->calc_String();
	    break;
	}
	case 1:
	{
	    return m_indexable_identifier->calc_String(*m_scope_Indexes[0]);
	    break;
	}
	case 2:
	{
	    return m_indexable_identifier->calc_String(*m_scope_Indexes[0], *m_scope_Indexes[1]);
	    break;
	}
	case 3:
	{
	    return m_indexable_identifier->calc_String(*m_scope_Indexes[0], *m_scope_Indexes[1], *m_scope_Indexes[2]);
	    break;
	}
	default:
	{
	    return m_indexable_identifier->calc_String(m_scope_Indexes);
	    break;
	}
	}
    }


    int sSpecifiedIdentifier::calc_CNF(void) const
    {
	switch(m_scope_Indexes.size())
	{
	case 0:
	{
	    return m_indexable_identifier->calc_CNF();
	    break;
	}
	case 1:
	{
	    return m_indexable_identifier->calc_CNF(*m_scope_Indexes[0]);
	    break;
	}
	case 2:
	{
	    return m_indexable_identifier->calc_CNF(*m_scope_Indexes[0], *m_scope_Indexes[1]);
	    break;
	}
	case 3:
	{
	    return m_indexable_identifier->calc_CNF(*m_scope_Indexes[0], *m_scope_Indexes[1], *m_scope_Indexes[2]);
	    break;
	}
	default:
	{
	    return m_indexable_identifier->calc_CNF(m_scope_Indexes);
	    break;
	}
	}
    }
    

/*----------------------------------------------------------------------------*/
// sIndexableIdentifier

    sIndexableIdentifier::sIndexableIdentifier()
	: m_variable_store(NULL)
    {
	// nothing
    }

    sIndexableIdentifier::sIndexableIdentifier(sVariableStore_CNF *variable_store, const sString &base_name)
	: m_variable_store(variable_store)
	, m_base_name(base_name)
    {
	m_first_cnf_variable = variable_store->get_Last_CNFVariable();
	m_N_cnf_Variables = 1;
	variable_store->alloc_CNFVariables(1);
    }


    sIndexableIdentifier::sIndexableIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, const sIndexScope &scope)
	: m_variable_store(variable_store)
	, m_base_name(base_name)
    {
	m_first_cnf_variable = variable_store->get_Last_CNFVariable();
	m_N_cnf_Variables = scope.get_Size();
	variable_store->alloc_CNFVariables(m_N_cnf_Variables);

	m_index_Scopes.push_back(scope.clone());
    }


    sIndexableIdentifier::sIndexableIdentifier(sVariableStore_CNF *variable_store,
					       const sString      &base_name,
					       const sIndexScope  &scope_1,
					       const sIndexScope  &scope_2)
	: m_variable_store(variable_store)
	, m_base_name(base_name)
    {
	m_first_cnf_variable = variable_store->get_Last_CNFVariable();
	m_N_cnf_Variables = scope_1.get_Size() * scope_2.get_Size();
	variable_store->alloc_CNFVariables(m_N_cnf_Variables);

	m_index_Scopes.push_back(scope_1.clone());
	m_index_Scopes.push_back(scope_2.clone());
    }


    sIndexableIdentifier::sIndexableIdentifier(sVariableStore_CNF *variable_store,
					       const sString      &base_name,
					       const sIndexScope  &scope_1,
					       const sIndexScope  &scope_2,
					       const sIndexScope  &scope_3)
	: m_variable_store(variable_store)
	, m_base_name(base_name)
    {
	m_first_cnf_variable = variable_store->get_Last_CNFVariable();
	m_N_cnf_Variables = scope_1.get_Size() * scope_2.get_Size() * scope_3.get_Size();
	variable_store->alloc_CNFVariables(m_N_cnf_Variables);

	m_index_Scopes.push_back(scope_1.clone());
	m_index_Scopes.push_back(scope_2.clone());
	m_index_Scopes.push_back(scope_3.clone());
    }


    sIndexableIdentifier::sIndexableIdentifier(sVariableStore_CNF       *variable_store,
					       const sString             &base_name,
					       const IndexScopes_vector &index_Scopes)
	: m_variable_store(variable_store)
	, m_base_name(base_name)
    {
	m_first_cnf_variable = variable_store->get_Last_CNFVariable();
	m_N_cnf_Variables = 1;

	for (IndexScopes_vector::const_iterator scope = index_Scopes.begin(); scope != index_Scopes.end(); ++scope)
	{
	    m_N_cnf_Variables *= (*scope)->get_Size();
	    sIndexScope *index_scope = (*scope)->clone();
	    m_index_Scopes.push_back(index_scope);
	}
	variable_store->alloc_CNFVariables(m_N_cnf_Variables);
    }


    sIndexableIdentifier::sIndexableIdentifier(const sIndexableIdentifier &indexable_identifier)
	: m_variable_store(indexable_identifier.m_variable_store)
	, m_base_name(indexable_identifier.m_base_name)
	, m_first_cnf_variable(indexable_identifier.m_first_cnf_variable)
	, m_N_cnf_Variables(indexable_identifier.m_N_cnf_Variables)
	  
    {
	for (IndexScopes_vector::const_iterator scope = indexable_identifier.m_index_Scopes.begin(); scope != indexable_identifier.m_index_Scopes.end(); ++scope)
	{
	    sIndexScope *index_scope = (*scope)->clone();
	    m_index_Scopes.push_back(index_scope);
	}	    
    }


    const sIndexableIdentifier& sIndexableIdentifier::operator=(const sIndexableIdentifier &indexable_identifier)
    {
	m_variable_store = indexable_identifier.m_variable_store;
	m_base_name = indexable_identifier.m_base_name;

	m_first_cnf_variable = indexable_identifier.m_first_cnf_variable;
	m_N_cnf_Variables = indexable_identifier.m_N_cnf_Variables;

	m_first_cnf_variable = indexable_identifier.m_first_cnf_variable;
	m_N_cnf_Variables = indexable_identifier.m_N_cnf_Variables;

	for (IndexScopes_vector::const_iterator scope = m_index_Scopes.begin(); scope != m_index_Scopes.end(); ++scope)
	{
	    delete *scope;
	}

	m_index_Scopes.clear();

	for (IndexScopes_vector::const_iterator scope = indexable_identifier.m_index_Scopes.begin(); scope != indexable_identifier.m_index_Scopes.end(); ++scope)
	{
	    sIndexScope *index_scope = (*scope)->clone();
	    m_index_Scopes.push_back(index_scope);
	}	    
	
	return *this;
    }


    sIndexableIdentifier::~sIndexableIdentifier()
    {
	for (IndexScopes_vector::const_iterator scope = m_index_Scopes.begin(); scope != m_index_Scopes.end(); ++scope)
	{
	    delete *scope;
	}
    }


    sString sIndexableIdentifier::calc_String(void) const
    {
	return m_base_name;
    }


    sString sIndexableIdentifier::calc_String(const sIndex &index) const
    {
	sASSERT(index >= *m_index_Scopes[0]->get_Begin() && index < *m_index_Scopes[0]->get_End());

	return m_base_name + "_" + index.to_String();
    }


    sString sIndexableIdentifier::calc_String(const sIndex &index_1, const sIndex &index_2) const
    {
	sASSERT(index_1 >= *m_index_Scopes[0]->get_Begin() && index_1 < *m_index_Scopes[0]->get_End());
	sASSERT(index_2 >= *m_index_Scopes[1]->get_Begin() && index_2 < *m_index_Scopes[1]->get_End());

	return m_base_name + "_" + index_1.to_String() + "^" + index_2.to_String();
    }


    sString sIndexableIdentifier::calc_String(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const
    {
	sASSERT(index_1 >= *m_index_Scopes[0]->get_Begin() && index_1 < *m_index_Scopes[0]->get_End());
	sASSERT(index_2 >= *m_index_Scopes[1]->get_Begin() && index_2 < *m_index_Scopes[1]->get_End());
	sASSERT(index_3 >= *m_index_Scopes[2]->get_Begin() && index_3 < *m_index_Scopes[2]->get_End());

	return m_base_name + "_" + index_1.to_String() + "^" + index_2.to_String() + "/" + index_3.to_String();;
    }


    sString sIndexableIdentifier::calc_String(const Indexes_vector &scope_Indexes) const
    {
	sString output = m_base_name;

	Indexes_vector::const_iterator index = scope_Indexes.begin();

	for (IndexScopes_vector::const_iterator scope = m_index_Scopes.begin(); scope != m_index_Scopes.end(); ++scope, ++index)
	{
	    sASSERT(**index >= *(*scope)->get_Begin() && **index < *(*scope)->get_End());
	    output += "." + (*index)->to_String();
	}

	return output;
    }


    int sIndexableIdentifier::calc_CNF(void) const
    {
	return m_first_cnf_variable;
    }


    int sIndexableIdentifier::calc_CNF(const sIndex &index) const
    {
	sASSERT(index >= *m_index_Scopes[0]->get_Begin() && index < *m_index_Scopes[0]->get_End());

	return m_first_cnf_variable + (index - *m_index_Scopes[0]->get_Begin());
    }


    int sIndexableIdentifier::calc_CNF(const sIndex &index_1, const sIndex &index_2) const
    {
	sASSERT(index_1 >= *m_index_Scopes[0]->get_Begin() && index_1 < *m_index_Scopes[0]->get_End());
	sASSERT(index_2 >= *m_index_Scopes[1]->get_Begin() && index_2 < *m_index_Scopes[1]->get_End());

	return m_first_cnf_variable + (index_1 - *m_index_Scopes[0]->get_Begin()) * m_index_Scopes[1]->get_Size() + (index_2 - *m_index_Scopes[1]->get_Begin());
    }


    int sIndexableIdentifier::calc_CNF(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const
    {
	sASSERT(index_1 >= *m_index_Scopes[0]->get_Begin() && index_1 < *m_index_Scopes[0]->get_End());
	sASSERT(index_2 >= *m_index_Scopes[1]->get_Begin() && index_2 < *m_index_Scopes[1]->get_End());
	sASSERT(index_3 >= *m_index_Scopes[2]->get_Begin() && index_3 < *m_index_Scopes[2]->get_End());

	return m_first_cnf_variable + (index_1 - *m_index_Scopes[0]->get_Begin()) * m_index_Scopes[1]->get_Size() * m_index_Scopes[2]->get_Size()
                         	    + (index_2 - *m_index_Scopes[1]->get_Begin()) * m_index_Scopes[2]->get_Size()
                                    + (index_3 - *m_index_Scopes[2]->get_Begin());
    }


    int sIndexableIdentifier::calc_CNF(const Indexes_vector &scope_Indexes) const
    {
	int offset = m_N_cnf_Variables;
	int cnf_var = m_first_cnf_variable;

	Indexes_vector::const_iterator index = scope_Indexes.begin();

	for (IndexScopes_vector::const_iterator scope = m_index_Scopes.begin(); scope != m_index_Scopes.end(); ++scope, ++index)
	{
	    sASSERT(**index >= *(*scope)->get_Begin() && **index < *(*scope)->get_End());
	    offset /= (*scope)->get_Size();
	    cnf_var += (**index - *(*scope)->get_Begin()) * offset; 
	}

	return cnf_var;
    }

    sSpecifiedIdentifier sIndexableIdentifier::translate_CNFVariable(int cnf_variable) const
    {	
	sASSERT(m_variable_store != NULL);

	if (cnf_variable >= m_first_cnf_variable && cnf_variable < m_first_cnf_variable + m_N_cnf_Variables)
	{
	    int offset = m_N_cnf_Variables;
	    cnf_variable -= m_first_cnf_variable;
	    Indexes_vector identifier_Indexes;

	    for (IndexScopes_vector::const_iterator scope = m_index_Scopes.begin(); scope != m_index_Scopes.end(); ++scope)
	    {
		offset /= (*scope)->get_Size();
		sIndex_iterator index_iter = (*scope)->get_Begin();
		index_iter += (cnf_variable / offset);

		identifier_Indexes.push_back((*index_iter).clone());
		cnf_variable %= offset;
	    }
	    sSpecifiedIdentifier specified_identifier(this, identifier_Indexes);

	    for (Indexes_vector::iterator index = identifier_Indexes.begin(); index != identifier_Indexes.end(); ++index)
	    {
		delete *index;
	    }
	    return specified_identifier;
	}
	else
	{
	    return sSpecifiedIdentifier();
	}
    }


    void sIndexableIdentifier::to_Screen(const sString &indent) const
    {
	printf("%sIndexable identifier (type = ", indent.c_str());

	switch (get_Arity())
	{
	case 0:
	{
	    printf("0-ary) [\n");
	    break;
	}
	case 1:
	{
	    printf("unary) [\n");
	    break;
	}
	case 2:
	{
	    printf("binary) [\n");
	    break;
	}
	case 3:
	{
	    printf("ternary) [\n");
	    break;
	}
	default:
	{
	    printf("%d-ary) [\n", get_Arity());
	    break;
	}
	}

	switch (get_Arity())
	{
	case 0:
	{
	    printf("%s%s%s->%d\n", sRELOC_INDENT.c_str(), indent.c_str(), m_base_name.c_str(), m_first_cnf_variable);
	    break;
	}
	case 1:
	{
	    printf("%s%s", sRELOC_INDENT.c_str(), indent.c_str());
		
	    sIndex_iterator end = m_index_Scopes[0]->get_End();
	    for (sIndex_iterator index = m_index_Scopes[0]->get_Begin(); index != end; ++index)
	    {		
		sString identifier = calc_String(*index);
		int cnf_index =  calc_CNF(*index);
		printf("%s -> %d\t", identifier.c_str(), cnf_index);
	    }
	    printf("\n");
	    break;
	}
	case 2:
	{
	    sIndex_iterator end_1 = m_index_Scopes[0]->get_End();
	    for (sIndex_iterator index_1 = m_index_Scopes[0]->get_Begin(); index_1 != end_1; ++index_1)
	    {
		printf("%s%s", sRELOC_INDENT.c_str(), indent.c_str());

		sIndex_iterator end_2 = m_index_Scopes[1]->get_End();
		for (sIndex_iterator index_2 = m_index_Scopes[1]->get_Begin(); index_2 != end_2; ++index_2)
		{
		    sString identifier = calc_String(*index_1, *index_2);
		    int cnf_index =  calc_CNF(*index_1, *index_2);
		    printf("%s -> %d\t", identifier.c_str(), cnf_index);
		}
		printf("\n");
	    }
	    break;
	}
	case 3:
	{
	    sIndex_iterator end_1 = m_index_Scopes[0]->get_End();
	    sIndex_iterator pre_end_1 = --m_index_Scopes[0]->get_End();
	    for (sIndex_iterator index_1 = m_index_Scopes[0]->get_Begin(); index_1 != end_1; ++index_1)
	    {
		sIndex_iterator end_2 = m_index_Scopes[1]->get_End();
		for (sIndex_iterator index_2 = m_index_Scopes[1]->get_Begin(); index_2 != end_2; ++index_2)
		{
		    printf("%s%s", sRELOC_INDENT.c_str(), indent.c_str());

		    sIndex_iterator end_3 = m_index_Scopes[2]->get_End();
		    for (sIndex_iterator index_3 = m_index_Scopes[2]->get_Begin(); index_3 != end_3; ++index_3)
		    {
			sString identifier = calc_String(*index_1, *index_2, *index_3);
			int cnf_index =  calc_CNF(*index_1, *index_2, *index_3);
			printf("%s -> %d\t", identifier.c_str(), cnf_index);
		    }
		    printf("\n");
		}
		if (index_1 != pre_end_1)
		{
		    printf("\n");
		}
	    }
	    break;
	}
	default:
	{
	    Indexes_vector Indexes;
	    to_Screen_identifiers(Indexes, indent);
	    printf("\n");
	}
	}

	printf("%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/

    void sIndexableIdentifier::to_Screen_identifiers(const Indexes_vector &scope_Indexes, const sString &indent) const
    {
	if (scope_Indexes.size() == m_index_Scopes.size())
	{
	    sString output = m_base_name;
	    Indexes_vector::const_iterator index = scope_Indexes.begin();

	    for (IndexScopes_vector::const_iterator scope = m_index_Scopes.begin(); scope != m_index_Scopes.end(); ++scope, ++index)
	    {
		output += "." + (*index)->to_String();
	    }

	    printf("%s -> %d\t", output.c_str(), calc_CNF(scope_Indexes));
	}
	else
	{
	    if (m_index_Scopes.size() - scope_Indexes.size() == 1)
	    {
		printf("%s%s", indent.c_str(), sRELOC_INDENT.c_str());
	    }
	    const sIndexScope *scope = m_index_Scopes[scope_Indexes.size()];

	    sIndex_iterator end = scope->get_End();
	    for (sIndex_iterator index = scope->get_Begin(); index != end; ++index)
	    {
		Indexes_vector scope_Indexes_rec = scope_Indexes;
		scope_Indexes_rec.push_back(&(*index));
		to_Screen_identifiers(scope_Indexes_rec, indent);
	    }

	    bool line = false;
	    int last_index = scope_Indexes.size() - 1;

	    for (int i = 0; i <= last_index ; ++i)
	    {
		if (*scope_Indexes[i] != *(--m_index_Scopes[i]->get_End()))
		{
		    line = true;
		    break;
		}
	    }	
	    if (line)
	    {
		printf("\n");
	    }
	}
    }


/*----------------------------------------------------------------------------*/
// sSpecifiedIStatedentifier

    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier()
	: m_state_identifier(NULL)
    {
	// nothong
    }


    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier)
	: m_state_identifier(state_identifier)
    {
	// nothing
    }


    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const sIndex &index)
	: m_state_identifier(state_identifier)
    {
	m_scope_Indexes.push_back(index.clone());
    }


    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const sIndex &index_1, const sIndex &index_2)
	: m_state_identifier(state_identifier)
    {
	m_scope_Indexes.push_back(index_1.clone());
	m_scope_Indexes.push_back(index_2.clone());
    }


    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const sIndex &index_1, const sIndex &index_2, const sIndex &index_3)
	: m_state_identifier(state_identifier)
    {
	m_scope_Indexes.push_back(index_1.clone());
	m_scope_Indexes.push_back(index_2.clone());
	m_scope_Indexes.push_back(index_3.clone());
    }


    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const Indexes_vector &scope_Indexes)
	: m_state_identifier(state_identifier)
    {
	for (Indexes_vector::const_iterator index = scope_Indexes.begin(); index != scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}
    }
    

    sSpecifiedStateIdentifier::sSpecifiedStateIdentifier(const sSpecifiedStateIdentifier &specified_identifier)
	: m_state_identifier(specified_identifier.m_state_identifier)
    {
	for (Indexes_vector::const_iterator index = specified_identifier.m_scope_Indexes.begin(); index != specified_identifier.m_scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}
    }


    const sSpecifiedStateIdentifier& sSpecifiedStateIdentifier::operator=(const sSpecifiedStateIdentifier &specified_identifier)
    {
	m_state_identifier = specified_identifier.m_state_identifier;

	for (Indexes_vector::const_iterator index = m_scope_Indexes.begin(); index != m_scope_Indexes.end(); ++index)
	{
	    delete *index;
	}

	m_scope_Indexes.clear();

	for (Indexes_vector::const_iterator index = specified_identifier.m_scope_Indexes.begin(); index != specified_identifier.m_scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}

	return *this;
    }


    sSpecifiedStateIdentifier::~sSpecifiedStateIdentifier()
    {
	for (Indexes_vector::const_iterator index = m_scope_Indexes.begin(); index != m_scope_Indexes.end(); ++index)
	{
	    delete *index;
	}
    }
    

    sString sSpecifiedStateIdentifier::calc_String(int bit) const
    {
	switch(m_scope_Indexes.size())
	{
	case 0:
	{
	    return m_state_identifier->calc_String(bit);
	    break;
	}
	case 1:
	{
	    return m_state_identifier->calc_String(*m_scope_Indexes[0], bit);
	    break;
	}
	case 2:
	{
	    return m_state_identifier->calc_String(*m_scope_Indexes[0], *m_scope_Indexes[1], bit);
	    break;
	}
	case 3:
	{
	    return m_state_identifier->calc_String(*m_scope_Indexes[0], *m_scope_Indexes[1], *m_scope_Indexes[2], bit);
	    break;
	}
	default:
	{
	    return m_state_identifier->calc_String(m_scope_Indexes, bit);
	    break;
	}
	}
    }


    int sSpecifiedStateIdentifier::calc_CNF(int bit) const
    {
	switch(m_scope_Indexes.size())
	{
	case 0:
	{
	    return m_state_identifier->calc_CNF(bit);
	    break;
	}
	case 1:
	{
	    return m_state_identifier->calc_CNF(*m_scope_Indexes[0], bit);
	    break;
	}
	case 2:
	{
	    return m_state_identifier->calc_CNF(*m_scope_Indexes[0], *m_scope_Indexes[1], bit);
	    break;
	}
	case 3:
	{
	    return m_state_identifier->calc_CNF(*m_scope_Indexes[0], *m_scope_Indexes[1], *m_scope_Indexes[2], bit);
	    break;
	}
	default:
	{
	    return m_state_identifier->calc_CNF(m_scope_Indexes, bit);
	    break;
	}
	}
    }
    

/*----------------------------------------------------------------------------*/
// sIndexableStateIdentifier

    sIndexableStateIdentifier::sIndexableStateIdentifier()
    {
	// nothing
    }
    
    sIndexableStateIdentifier::sIndexableStateIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, int N_States)
	: m_N_States(N_States)
	, m_indexable_identifier(variable_store, base_name, sIntegerScope(0, (m_log_N_States = calc_Log2(N_States)) - 1))
    {
	// nothing
    }


    sIndexableStateIdentifier::sIndexableStateIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, int N_States, const sIndexScope &scope)
	: m_N_States(N_States)
	, m_indexable_identifier(variable_store, base_name, scope, sIntegerScope(0, (m_log_N_States = calc_Log2(N_States)) - 1))
    {
	// nothing
    }


    sIndexableStateIdentifier::sIndexableStateIdentifier(sVariableStore_CNF *variable_store,
							 const sString      &base_name,
							 int                 N_States,
							 const sIndexScope  &scope_1,
							 const sIndexScope  &scope_2)
	: m_N_States(N_States)
	, m_indexable_identifier(variable_store, base_name, scope_1, scope_2, sIntegerScope(0, (m_log_N_States = calc_Log2(N_States)) - 1))
    {
	// nothing
    }


    sIndexableStateIdentifier::sIndexableStateIdentifier(sVariableStore_CNF *variable_store,
							 const sString      &base_name,
							 int                 N_States,
							 const sIndexScope  &scope_1,
							 const sIndexScope  &scope_2,
							 const sIndexScope  &scope_3)
	: m_N_States(N_States)
    {
	IndexScopes_vector idx_Scopes;
	sIntegerScope state_scope(0, (m_log_N_States = calc_Log2(N_States)) - 1);
	idx_Scopes.push_back(&scope_1);
	idx_Scopes.push_back(&scope_2);
	idx_Scopes.push_back(&scope_3);
	idx_Scopes.push_back(&state_scope);

	m_indexable_identifier = sIndexableIdentifier(variable_store, base_name, idx_Scopes);
    }


    sIndexableStateIdentifier::sIndexableStateIdentifier(sVariableStore_CNF       *variable_store,
							 const sString            &base_name,
							 int                       N_States,
							 const IndexScopes_vector &index_Scopes)
	: m_N_States(N_States)
    {
	IndexScopes_vector idx_Scopes = index_Scopes;
	sIntegerScope state_scope(0, (m_log_N_States = calc_Log2(N_States)) - 1);
	idx_Scopes.push_back(&state_scope);

	m_indexable_identifier = sIndexableIdentifier(variable_store, base_name, idx_Scopes);
    }


    sIndexableStateIdentifier::sIndexableStateIdentifier(const sIndexableStateIdentifier &state_identifier)
	: m_N_States(state_identifier.m_N_States)
	, m_log_N_States(state_identifier.m_log_N_States)
	, m_indexable_identifier(state_identifier.m_indexable_identifier)
    {
	// nothing
    }


    const sIndexableStateIdentifier& sIndexableStateIdentifier::operator=(const sIndexableStateIdentifier &state_identifier)
    {
	m_N_States = state_identifier.m_N_States;
	m_log_N_States = state_identifier.m_log_N_States;
	m_indexable_identifier = state_identifier.m_indexable_identifier;

	return *this;
    }


    sIndexableStateIdentifier::~sIndexableStateIdentifier()
    {
	// nothing
    }


    sString sIndexableStateIdentifier::calc_String(int bit) const
    {
	return m_indexable_identifier.calc_String(sIntegerIndex(bit));
    }
    
    
    sString sIndexableStateIdentifier::calc_String(const sIndex &index, int bit) const
    {
	return m_indexable_identifier.calc_String(index, sIntegerIndex(bit));
    }
    
    
    sString sIndexableStateIdentifier::calc_String(const sIndex &index_1, const sIndex &index_2, int bit) const
    {
	return m_indexable_identifier.calc_String(index_1, index_2, sIntegerIndex(bit));
    }
    
    
    sString sIndexableStateIdentifier::calc_String(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3, int bit) const
    {
	Indexes_vector scope_Indexes;
	scope_Indexes.push_back(&index_1);
	scope_Indexes.push_back(&index_2);
	scope_Indexes.push_back(&index_3);

	sIntegerIndex bit_index(bit);
	scope_Indexes.push_back(&bit_index);

	return m_indexable_identifier.calc_String(scope_Indexes);
    }
    
    
    sString sIndexableStateIdentifier::calc_String(const Indexes_vector &scope_Indexes, int bit) const
    {
	Indexes_vector scope_Idxs = scope_Indexes;
	sIntegerIndex bit_index(bit);
	scope_Idxs.push_back(&bit_index);

	return m_indexable_identifier.calc_String(scope_Idxs);
    }
    

    int sIndexableStateIdentifier::calc_CNF(int bit) const
    {
	return m_indexable_identifier.calc_CNF(sIntegerIndex(bit));
    }
    
    
    int sIndexableStateIdentifier::calc_CNF(const sIndex &index, int bit) const
    {
	return m_indexable_identifier.calc_CNF(index, sIntegerIndex(bit));
    }
    
    
    int sIndexableStateIdentifier::calc_CNF(const sIndex &index_1, const sIndex &index_2, int bit) const
    {
	return m_indexable_identifier.calc_CNF(index_1, index_2, sIntegerIndex(bit));
    }
    
    
    int sIndexableStateIdentifier::calc_CNF(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3, int bit) const
    {
	Indexes_vector scope_Indexes;
	scope_Indexes.push_back(&index_1);
	scope_Indexes.push_back(&index_2);
	scope_Indexes.push_back(&index_3);

	sIntegerIndex bit_index(bit);
	scope_Indexes.push_back(&bit_index);

	return m_indexable_identifier.calc_CNF(scope_Indexes);
    }


    int sIndexableStateIdentifier::calc_CNF(const Indexes_vector &scope_Indexes, int bit) const
    {
	Indexes_vector scope_Idxs = scope_Indexes;
	sIntegerIndex bit_index(bit);
	scope_Idxs.push_back(&bit_index);

	return m_indexable_identifier.calc_CNF(scope_Idxs);
    }


    sSpecifiedIdentifier sIndexableStateIdentifier::translate_CNFVariable(int cnf_variable) const
    {
	return m_indexable_identifier.translate_CNFVariable(cnf_variable);
    }


    void sIndexableStateIdentifier::to_Screen(const sString &indent) const
    {
	printf("%sIndexable identifier (|States| = %d, log_2|States| = %d) [\n", indent.c_str(), m_N_States, m_log_N_States);
	m_indexable_identifier.to_Screen(indent + sRELOC_INDENT);

	printf("%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/
    
    int sIndexableStateIdentifier::calc_Log2(int value)
    {
	int log = 0;
	int v = 1;

	while (v < value)
	{
	    v *= 2;
	    ++log;
	}

	return log;
    }

    int sIndexableStateIdentifier::calc_Exp2(int value)
    {
	int exp = 1;

	while (value > 0)
	{
	    exp *= 2;
	    --value;
	}

	return exp;
    }




/*----------------------------------------------------------------------------*/
// sSpecifiedIBitdentifier

    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier()
	: m_bit_identifier(NULL)
    {
	// nothong
    }


    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier)
	: m_bit_identifier(bit_identifier)
    {
	// nothing
    }


    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const sIndex &index)
	: m_bit_identifier(bit_identifier)
    {
	m_scope_Indexes.push_back(index.clone());
    }


    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const sIndex &index_1, const sIndex &index_2)
	: m_bit_identifier(bit_identifier)
    {
	m_scope_Indexes.push_back(index_1.clone());
	m_scope_Indexes.push_back(index_2.clone());
    }


    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const sIndex &index_1, const sIndex &index_2, const sIndex &index_3)
	: m_bit_identifier(bit_identifier)
    {
	m_scope_Indexes.push_back(index_1.clone());
	m_scope_Indexes.push_back(index_2.clone());
	m_scope_Indexes.push_back(index_3.clone());
    }


    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const Indexes_vector &scope_Indexes)
	: m_bit_identifier(bit_identifier)
    {
	for (Indexes_vector::const_iterator index = scope_Indexes.begin(); index != scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}
    }
    

    sSpecifiedBitIdentifier::sSpecifiedBitIdentifier(const sSpecifiedBitIdentifier &specified_identifier)
	: m_bit_identifier(specified_identifier.m_bit_identifier)
    {
	for (Indexes_vector::const_iterator index = specified_identifier.m_scope_Indexes.begin(); index != specified_identifier.m_scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}
    }


    const sSpecifiedBitIdentifier& sSpecifiedBitIdentifier::operator=(const sSpecifiedBitIdentifier &specified_identifier)
    {
	m_bit_identifier = specified_identifier.m_bit_identifier;

	for (Indexes_vector::const_iterator index = m_scope_Indexes.begin(); index != m_scope_Indexes.end(); ++index)
	{
	    delete *index;
	}

	m_scope_Indexes.clear();

	for (Indexes_vector::const_iterator index = specified_identifier.m_scope_Indexes.begin(); index != specified_identifier.m_scope_Indexes.end(); ++index)
	{
	    m_scope_Indexes.push_back((*index)->clone());
	}

	return *this;
    }


    sSpecifiedBitIdentifier::~sSpecifiedBitIdentifier()
    {
	for (Indexes_vector::const_iterator index = m_scope_Indexes.begin(); index != m_scope_Indexes.end(); ++index)
	{
	    delete *index;
	}
    }
    

    sString sSpecifiedBitIdentifier::calc_String(void) const
    {
	switch(m_scope_Indexes.size())
	{
	case 0:
	{
	    return m_bit_identifier->calc_String();
	    break;
	}
	case 1:
	{
	    return m_bit_identifier->calc_String(*m_scope_Indexes[0]);
	    break;
	}
	case 2:
	{
	    return m_bit_identifier->calc_String(*m_scope_Indexes[0], *m_scope_Indexes[1]);
	    break;
	}
	case 3:
	{
	    return m_bit_identifier->calc_String(*m_scope_Indexes[0], *m_scope_Indexes[1], *m_scope_Indexes[2]);
	    break;
	}
	default:
	{
	    return m_bit_identifier->calc_String(m_scope_Indexes);
	    break;
	}
	}
    }


    int sSpecifiedBitIdentifier::calc_CNF(void) const
    {
	switch(m_scope_Indexes.size())
	{
	case 0:
	{
	    return m_bit_identifier->calc_CNF();
	    break;
	}
	case 1:
	{
	    return m_bit_identifier->calc_CNF(*m_scope_Indexes[0]);
	    break;
	}
	case 2:
	{
	    return m_bit_identifier->calc_CNF(*m_scope_Indexes[0], *m_scope_Indexes[1]);
	    break;
	}
	case 3:
	{
	    return m_bit_identifier->calc_CNF(*m_scope_Indexes[0], *m_scope_Indexes[1], *m_scope_Indexes[2]);
	    break;
	}
	default:
	{
	    return m_bit_identifier->calc_CNF(m_scope_Indexes);
	    break;
	}
	}
    }
    

/*----------------------------------------------------------------------------*/
// sIndexableBitIdentifier

    sIndexableBitIdentifier::sIndexableBitIdentifier()
    {
	// nothing
    }
    
    sIndexableBitIdentifier::sIndexableBitIdentifier(sVariableStore_CNF *variable_store, const sString &base_name)
	: m_indexable_identifier(variable_store, base_name)
    {
	// nothing
    }


    sIndexableBitIdentifier::sIndexableBitIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, const sIndexScope &scope)
	: m_indexable_identifier(variable_store, base_name, scope)
    {
	// nothing
    }


    sIndexableBitIdentifier::sIndexableBitIdentifier(sVariableStore_CNF *variable_store,
						     const sString      &base_name,
						     const sIndexScope  &scope_1,
						     const sIndexScope  &scope_2)
	: m_indexable_identifier(variable_store, base_name, scope_1, scope_2)
    {
	// nothing
    }


    sIndexableBitIdentifier::sIndexableBitIdentifier(sVariableStore_CNF *variable_store,
						     const sString      &base_name,
						     const sIndexScope  &scope_1,
						     const sIndexScope  &scope_2,
						     const sIndexScope  &scope_3)
    {
	IndexScopes_vector idx_Scopes;
	idx_Scopes.push_back(&scope_1);
	idx_Scopes.push_back(&scope_2);
	idx_Scopes.push_back(&scope_3);

	m_indexable_identifier = sIndexableIdentifier(variable_store, base_name, idx_Scopes);
    }


    sIndexableBitIdentifier::sIndexableBitIdentifier(sVariableStore_CNF       *variable_store,
						     const sString            &base_name,
						     const IndexScopes_vector &index_Scopes)
    {
	IndexScopes_vector idx_Scopes = index_Scopes;
	m_indexable_identifier = sIndexableIdentifier(variable_store, base_name, idx_Scopes);
    }


    sIndexableBitIdentifier::sIndexableBitIdentifier(const sIndexableBitIdentifier &bit_identifier)
	: m_indexable_identifier(bit_identifier.m_indexable_identifier)
    {
	// nothing
    }


    const sIndexableBitIdentifier& sIndexableBitIdentifier::operator=(const sIndexableBitIdentifier &bit_identifier)
    {
	m_indexable_identifier = bit_identifier.m_indexable_identifier;

	return *this;
    }


    sIndexableBitIdentifier::~sIndexableBitIdentifier()
    {
	// nothing
    }


    sString sIndexableBitIdentifier::calc_String(void) const
    {
	return m_indexable_identifier.calc_String();
    }
    
    
    sString sIndexableBitIdentifier::calc_String(const sIndex &index) const
    {
	return m_indexable_identifier.calc_String(index);
    }
    
    
    sString sIndexableBitIdentifier::calc_String(const sIndex &index_1, const sIndex &index_2) const
    {
	return m_indexable_identifier.calc_String(index_1, index_2);
    }
    
    
    sString sIndexableBitIdentifier::calc_String(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const
    {
	Indexes_vector scope_Indexes;
	scope_Indexes.push_back(&index_1);
	scope_Indexes.push_back(&index_2);
	scope_Indexes.push_back(&index_3);

	return m_indexable_identifier.calc_String(scope_Indexes);
    }
    
    
    sString sIndexableBitIdentifier::calc_String(const Indexes_vector &scope_Indexes) const
    {
	Indexes_vector scope_Idxs = scope_Indexes;
	return m_indexable_identifier.calc_String(scope_Idxs);
    }
    

    int sIndexableBitIdentifier::calc_CNF(void) const
    {
	return m_indexable_identifier.calc_CNF();
    }
    
    
    int sIndexableBitIdentifier::calc_CNF(const sIndex &index) const
    {
	return m_indexable_identifier.calc_CNF(index);
    }
    
    
    int sIndexableBitIdentifier::calc_CNF(const sIndex &index_1, const sIndex &index_2) const
    {
	return m_indexable_identifier.calc_CNF(index_1, index_2);
    }
    
    
    int sIndexableBitIdentifier::calc_CNF(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const
    {
	Indexes_vector scope_Indexes;
	scope_Indexes.push_back(&index_1);
	scope_Indexes.push_back(&index_2);
	scope_Indexes.push_back(&index_3);

	return m_indexable_identifier.calc_CNF(scope_Indexes);
    }


    int sIndexableBitIdentifier::calc_CNF(const Indexes_vector &scope_Indexes) const
    {
	Indexes_vector scope_Idxs = scope_Indexes;
	return m_indexable_identifier.calc_CNF(scope_Idxs);
    }


    sSpecifiedIdentifier sIndexableBitIdentifier::translate_CNFVariable(int cnf_variable) const
    {
	return m_indexable_identifier.translate_CNFVariable(cnf_variable);
    }


    void sIndexableBitIdentifier::to_Screen(const sString &indent) const
    {
	printf("%sIndexable BIT identifier [\n", indent.c_str());
	m_indexable_identifier.to_Screen(indent + sRELOC_INDENT);

	printf("%s]\n", indent.c_str());
    }




/*----------------------------------------------------------------------------*/
// sStateClauseGenerator

    sStateClauseGenerator::sStateClauseGenerator(sVariableStore_CNF *variable_store)
	: m_variable_store(variable_store)
	, m_aux_Identifier_cnt(0)
    {
	m_auxiliary_Identifiers[sINT_32_MAX] = sIndexableIdentifier();
    }


    sStateClauseGenerator::sStateClauseGenerator(const sStateClauseGenerator &clause_generator)
	: m_variable_store(clause_generator.m_variable_store)
	, m_aux_Identifier_cnt(clause_generator.m_aux_Identifier_cnt)
	, m_auxiliary_Identifiers(clause_generator.m_auxiliary_Identifiers)
    {
	// nothing
    }


    const sStateClauseGenerator& sStateClauseGenerator::operator=(const sStateClauseGenerator &clause_generator)
    {
	m_variable_store = clause_generator.m_variable_store;
	m_aux_Identifier_cnt = clause_generator.m_aux_Identifier_cnt;
	m_auxiliary_Identifiers = clause_generator.m_auxiliary_Identifiers;

	return *this;
    }


    int sStateClauseGenerator::count_Alignment(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	int N_total_States = sIndexableStateIdentifier::calc_Exp2(N_Bits);
	int N_States = spec_identifier.get_StateIdentifier()->get_StateCount();
	total_Literal_cnt += (N_total_States - N_States) * N_Bits;

	return (N_total_States - N_States);
    }


    int sStateClauseGenerator::generate_Alignment(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string, int sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	int N_total_States = sIndexableStateIdentifier::calc_Exp2(N_Bits);
	int N_States = spec_identifier.get_StateIdentifier()->get_StateCount();

	for (int align_state = N_States; align_state < N_total_States; ++align_state)
	{
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		// printf("align_state:%d\n", align_state);
		if (calc_Bit(bit, align_state) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier.calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier.calc_CNF(bit));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier.calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier.calc_CNF(bit));
		    }
		}
	    }
	    ++Clause_cnt;
	    fprintf(fw, " 0\n");
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_Alignment(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int sUNUSED(weight))
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	int N_total_States = sIndexableStateIdentifier::calc_Exp2(N_Bits);
	int N_States = spec_identifier.get_StateIdentifier()->get_StateCount();

	for (int align_state = N_States; align_state < N_total_States; ++align_state)
	{
	    vector<int> Literals;
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (calc_Bit(bit, align_state) == 1)
		{
		    Literals.push_back(-spec_identifier.calc_CNF(bit));
		}
		else
		{
		    Literals.push_back(spec_identifier.calc_CNF(bit));
		}
	    }
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif
	    cast_Clause(solver, Literals);
	}
    }    


    int sStateClauseGenerator::count_Equality(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, int sUNUSED(state)) const
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	total_Literal_cnt += N_Bits;;

	return N_Bits;;
    }


    int sStateClauseGenerator::generate_Equality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, int state, bool string, int sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    int state_bit = calc_Bit(bit, state);
	    sASSERT(state_bit == 1 || state_bit == 0);

	    if (state_bit == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier.calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier.calc_CNF(bit));
		}
		++Clause_cnt;
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier.calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier.calc_CNF(bit));
		}
		++Clause_cnt;
	    }
	}
#ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
#endif	
	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_Equality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int state, int sUNUSED(weight))
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    int state_bit = calc_Bit(bit, state);
	    sASSERT(state_bit == 1 || state_bit == 0);

	    if (state_bit == 1)
	    {
		cast_Clause(solver, spec_identifier.calc_CNF(bit));
	    }
	    else
	    {
		cast_Clause(solver, -spec_identifier.calc_CNF(bit));
	    }
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif			    
	}
    }


    int sStateClauseGenerator::count_Disequality(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, int sUNUSED(state)) const
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	total_Literal_cnt += N_Bits;

	return 1;
    }


    int sStateClauseGenerator::generate_Disequality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, int state, bool string, int sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    int state_bit = calc_Bit(bit, state);
	    sASSERT(state_bit == 1 || state_bit == 0);

	    if (state_bit == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier.calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier.calc_CNF(bit));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier.calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier.calc_CNF(bit));
		}
	    }
	}
	fprintf(fw, "0\n");
	++Clause_cnt;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_Disequality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int state, int sUNUSED(weight))
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	std::vector<int> Literals;
	
	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    int state_bit = calc_Bit(bit, state);
	    sASSERT(state_bit == 1 || state_bit == 0);

	    if (state_bit == 1)
	    {
		Literals.push_back(-spec_identifier.calc_CNF(bit));
	    }
	    else
	    {
		Literals.push_back(spec_identifier.calc_CNF(bit));
	    }
	}
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	}
	#endif
    }    


    int sStateClauseGenerator::count_Disequalities(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States) const
    {
	int Clause_cnt = 0;

	for (States_vector::const_iterator state = States.begin(); state != States.end(); ++state)
	{
	    Clause_cnt += count_Disequality(aux_Variable_cnt, total_Literal_cnt, spec_identifier, *state);
	}
	return Clause_cnt;
    }


    int sStateClauseGenerator::generate_Disequalities(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, bool string, int weight)
    {
	int Clause_cnt = 0;

	for (States_vector::const_iterator state = States.begin(); state != States.end(); ++state)
	{
	    Clause_cnt += generate_Disequality(fw, spec_identifier, *state, string, weight);
	}
	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_Disequalities(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, int weight)
    {
	for (States_vector::const_iterator state = States.begin(); state != States.end(); ++state)
	{
	    cast_Disequality(solver, spec_identifier, *state, weight);
	}
    }    


    int sStateClauseGenerator::count_DisjunctiveDisequality(int                             &sUNUSED(aux_Variable_cnt),
							    int                             &total_Literal_cnt,
							    const sSpecifiedStateIdentifier &spec_identifier_A,
							    int                              sUNUSED(state_A),
							    const sSpecifiedStateIdentifier &spec_identifier_B,
							    int                              sUNUSED(state_B)) const
    {
	int N_Bits_A = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_Bits_B = spec_identifier_B.get_StateIdentifier()->get_Log2_StateCount();

	total_Literal_cnt += N_Bits_A + N_Bits_B;

	return 1;
    }


    int sStateClauseGenerator::generate_DisjunctiveDisequality(FILE                            *fw,
							       const sSpecifiedStateIdentifier &spec_identifier_A,
							       int                              state_A,
							       const sSpecifiedStateIdentifier &spec_identifier_B,
							       int                              state_B,
							       bool                             string,
							       int                              sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits_A = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_A = 0; bit_A < N_Bits_A; ++bit_A)
	{
	    int state_bit_A = calc_Bit(bit_A, state_A);
	    sASSERT(state_bit_A == 1 || state_bit_A == 0);

	    if (state_bit_A == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_A.calc_String(bit_A).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_A.calc_CNF(bit_A));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_A.calc_String(bit_A).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_A.calc_CNF(bit_A));
		}
	    }
	}

	int N_Bits_B = spec_identifier_B.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_B = 0; bit_B < N_Bits_B; ++bit_B)
	{
	    int state_bit_B = calc_Bit(bit_B, state_B);
	    sASSERT(state_bit_B == 1 || state_bit_B == 0);

	    if (state_bit_B == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_B.calc_String(bit_B).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_B.calc_CNF(bit_B));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_B.calc_String(bit_B).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_B.calc_CNF(bit_B));
		}
	    }
	}
	fprintf(fw, "0\n");
	++Clause_cnt;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_DisjunctiveDisequality(Glucose::Solver                 *solver,
							    const sSpecifiedStateIdentifier &spec_identifier_A,
							    int                              state_A,
							    const sSpecifiedStateIdentifier &spec_identifier_B,
							    int                              state_B,
							    int                              sUNUSED(weight))
    {
	int N_Bits_A = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	std::vector<int> Literals;
	
	for (int bit_A = 0; bit_A < N_Bits_A; ++bit_A)
	{
	    int state_bit_A = calc_Bit(bit_A, state_A);
	    sASSERT(state_bit_A == 1 || state_bit_A == 0);

	    if (state_bit_A == 1)
	    {
		Literals.push_back(-spec_identifier_A.calc_CNF(bit_A));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_A.calc_CNF(bit_A));
	    }
	}

	int N_Bits_B = spec_identifier_B.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_B = 0; bit_B < N_Bits_B; ++bit_B)
	{
	    int state_bit_B = calc_Bit(bit_B, state_B);
	    sASSERT(state_bit_B == 1 || state_bit_B == 0);

	    if (state_bit_B == 1)
	    {
		Literals.push_back(-spec_identifier_B.calc_CNF(bit_B));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_B.calc_CNF(bit_B));
	    }
	}
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	}
	#endif
    }    


    int sStateClauseGenerator::count_Equality(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier_A, const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_B)) const
    {
	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	total_Literal_cnt += 4 * N_Bits;
	return 2 * N_Bits;
    }


    int sStateClauseGenerator::generate_Equality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier_A, const sSpecifiedStateIdentifier &spec_identifier_B, bool string, int sUNUSED(weight))
    {
	sASSERT(spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_B.get_StateIdentifier()->get_Log2_StateCount());

	int Clause_cnt = 0;
	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{	   
	    if (string)
	    {
		fprintf(fw, "-%s %s 0\n", spec_identifier_A.calc_String(bit).c_str(), spec_identifier_B.calc_String(bit).c_str());
		fprintf(fw, "%s -%s 0\n", spec_identifier_A.calc_String(bit).c_str(), spec_identifier_B.calc_String(bit).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d %d 0\n", spec_identifier_A.calc_CNF(bit), spec_identifier_B.calc_CNF(bit));
		fprintf(fw, "%d -%d 0\n", spec_identifier_A.calc_CNF(bit), spec_identifier_B.calc_CNF(bit));
	    }
	    Clause_cnt += 2;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_Equality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier_A, const sSpecifiedStateIdentifier &spec_identifier_B, int sUNUSED(weight))
    {
	sASSERT(spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_B.get_StateIdentifier()->get_Log2_StateCount());

	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{	   
	    cast_Clause(solver, -spec_identifier_A.calc_CNF(bit), spec_identifier_B.calc_CNF(bit));
	    cast_Clause(solver, spec_identifier_A.calc_CNF(bit), -spec_identifier_B.calc_CNF(bit));
		
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	    }	    
	}
	#endif
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_IF_Bits + N_THEN_Bits;
	total_Literal_cnt += N_IF_Bits * N_THEN_Bits + N_THEN_Bits + 12 * (N_IF_Bits + N_THEN_Bits);

	return (4 * N_IF_Bits + 5 * N_THEN_Bits);
    }


    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_IF_B.get_StateIdentifier()->get_Log2_StateCount())
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier not_equal_auxiliary(m_variable_store, "neq_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_IF_Bits - 1));
	m_auxiliary_Identifiers[not_equal_auxiliary.get_First_CNFVariable()] = not_equal_auxiliary;

	sIndexableIdentifier equal_auxiliary(m_variable_store, "eql_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_Bits - 1));
	m_auxiliary_Identifiers[equal_auxiliary.get_First_CNFVariable()] = equal_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (string)
		{
		    fprintf(fw, "%s ", not_equal_auxiliary.calc_String(sIntegerIndex(bit_2)).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", not_equal_auxiliary.calc_CNF(sIntegerIndex(bit_2)));
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_IF_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s -%s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s %s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s %s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d -%d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d %d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d %d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}

	for (int bit = 0; bit < N_THEN_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }

    
    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							 int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_IF_B.get_StateIdentifier()->get_Log2_StateCount())
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier not_equal_auxiliary(m_variable_store, "neq_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_IF_Bits - 1));
	m_auxiliary_Identifiers[not_equal_auxiliary.get_First_CNFVariable()] = not_equal_auxiliary;

	sIndexableIdentifier equal_auxiliary(m_variable_store, "eql_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_Bits - 1));
	m_auxiliary_Identifiers[equal_auxiliary.get_First_CNFVariable()] = equal_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		Literals.push_back(not_equal_auxiliary.calc_CNF(sIntegerIndex(bit_2)));
	    }
	    Literals.push_back(equal_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    	    	    
	}

	for (int bit = 0; bit < N_IF_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), -not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_IF_A.calc_CNF(bit), -spec_identifier_IF_B.calc_CNF(bit), -not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_IF_A.calc_CNF(bit), -spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    	    
	}

	for (int bit = 0; bit < N_THEN_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A.calc_CNF(bit), -spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), -equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_THEN_A.calc_CNF(bit), -spec_identifier_THEN_B.calc_CNF(bit), -equal_auxiliary.calc_CNF(sIntegerIndex(bit)));	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    
	}
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &sUNUSED(aux_Variable_cnt),
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              sUNUSED(state_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							 int                              sUNUSED(state_THEN_B)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	total_Literal_cnt += N_THEN_Bits * N_IF_Bits + N_THEN_Bits;

	return N_THEN_Bits;
    }

	
    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    int                              state_THEN_B,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              state_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							 int                              state_THEN_B,
							 int                              sUNUSED(weight))
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    
	}
    }
    
    
    int sStateClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              sUNUSED(state_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_THEN_Bits;
	total_Literal_cnt += N_THEN_Bits * N_IF_Bits + N_THEN_Bits + 12 * N_THEN_Bits;

	return (5 * N_THEN_Bits);
    }
	

    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_auxiliary(m_variable_store, "eql_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_Bits - 1));
	m_auxiliary_Identifiers[equal_auxiliary.get_First_CNFVariable()] = equal_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_THEN_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str(), equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              state_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							 int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_auxiliary(m_variable_store, "eql_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_Bits - 1));
	m_auxiliary_Identifiers[equal_auxiliary.get_First_CNFVariable()] = equal_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)	    
	{
	    std::vector<int> Literals;
	    
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    Literals.push_back(equal_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    	    
	}

	for (int bit = 0; bit < N_THEN_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A.calc_CNF(bit), -spec_identifier_THEN_B.calc_CNF(bit), equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit), -equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_THEN_A.calc_CNF(bit), -spec_identifier_THEN_B.calc_CNF(bit), -equal_auxiliary.calc_CNF(sIntegerIndex(bit)));

#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    
	}
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_1),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_2)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_IF_Bits + N_THEN_1_Bits + N_THEN_2_Bits;
	total_Literal_cnt += N_THEN_1_Bits * N_IF_Bits + N_THEN_1_Bits + N_THEN_2_Bits * N_IF_Bits + N_THEN_2_Bits + 12 * (N_IF_Bits + N_THEN_1_Bits + N_THEN_2_Bits);

	return (4 * N_IF_Bits + 5 * N_THEN_1_Bits + 5 * N_THEN_2_Bits);
    }


    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_IF_B.get_StateIdentifier()->get_Log2_StateCount())
	sASSERT(spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_1.get_StateIdentifier()->get_Log2_StateCount())
	sASSERT(spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_2.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier not_equal_auxiliary(m_variable_store, "neq_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_IF_Bits - 1));
	m_auxiliary_Identifiers[not_equal_auxiliary.get_First_CNFVariable()] = not_equal_auxiliary;

	sIndexableIdentifier equal_1_auxiliary(m_variable_store, "eql_1_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_1_Bits - 1));
	m_auxiliary_Identifiers[equal_1_auxiliary.get_First_CNFVariable()] = equal_1_auxiliary;

	sIndexableIdentifier equal_2_auxiliary(m_variable_store, "eql_2_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_2_Bits - 1));
	m_auxiliary_Identifiers[equal_2_auxiliary.get_First_CNFVariable()] = equal_2_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (string)
		{
		    fprintf(fw, "%s ", not_equal_auxiliary.calc_String(sIntegerIndex(bit_2)).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", not_equal_auxiliary.calc_CNF(sIntegerIndex(bit_2)));
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_1_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_1_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (string)
		{
		    fprintf(fw, "%s ", not_equal_auxiliary.calc_String(sIntegerIndex(bit_2)).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", not_equal_auxiliary.calc_CNF(sIntegerIndex(bit_2)));
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_2_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_2_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_IF_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s -%s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s %s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s %s 0\n", spec_identifier_IF_A.calc_String(bit).c_str(), spec_identifier_IF_B.calc_String(bit).c_str(), not_equal_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d -%d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d %d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d %d 0\n", spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}

	for (int bit = 0; bit < N_THEN_1_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A_1.calc_String(bit).c_str(), spec_identifier_THEN_B_1.calc_String(bit).c_str(), equal_1_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A_1.calc_String(bit).c_str(), spec_identifier_THEN_B_1.calc_String(bit).c_str(), equal_1_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A_1.calc_String(bit).c_str(), spec_identifier_THEN_B_1.calc_String(bit).c_str(), equal_1_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A_1.calc_String(bit).c_str(), spec_identifier_THEN_B_1.calc_String(bit).c_str(), equal_1_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit), spec_identifier_THEN_B_1.calc_CNF(bit), equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit), spec_identifier_THEN_B_1.calc_CNF(bit), equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit), spec_identifier_THEN_B_1.calc_CNF(bit), equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit), spec_identifier_THEN_B_1.calc_CNF(bit), equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}

	for (int bit = 0; bit < N_THEN_2_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_1,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
							 int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_IF_B.get_StateIdentifier()->get_Log2_StateCount())
	sASSERT(spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_1.get_StateIdentifier()->get_Log2_StateCount())
	sASSERT(spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_2.get_StateIdentifier()->get_Log2_StateCount())

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier not_equal_auxiliary(m_variable_store, "neq_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_IF_Bits - 1));
	m_auxiliary_Identifiers[not_equal_auxiliary.get_First_CNFVariable()] = not_equal_auxiliary;

	sIndexableIdentifier equal_1_auxiliary(m_variable_store, "eql_1_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_1_Bits - 1));
	m_auxiliary_Identifiers[equal_1_auxiliary.get_First_CNFVariable()] = equal_1_auxiliary;

	sIndexableIdentifier equal_2_auxiliary(m_variable_store, "eql_2_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_2_Bits - 1));
	m_auxiliary_Identifiers[equal_2_auxiliary.get_First_CNFVariable()] = equal_2_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		Literals.push_back(not_equal_auxiliary.calc_CNF(sIntegerIndex(bit_2)));
	    }
	    Literals.push_back(equal_1_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    	    
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		Literals.push_back(not_equal_auxiliary.calc_CNF(sIntegerIndex(bit_2)));
	    }
	    Literals.push_back(equal_2_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    	    
	}

	for (int bit = 0; bit < N_IF_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), -not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_IF_A.calc_CNF(bit), -spec_identifier_IF_B.calc_CNF(bit), -not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_IF_A.calc_CNF(bit), spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_IF_A.calc_CNF(bit), -spec_identifier_IF_B.calc_CNF(bit), not_equal_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    	    
	}

	for (int bit = 0; bit < N_THEN_1_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A_1.calc_CNF(bit), spec_identifier_THEN_B_1.calc_CNF(bit), equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_1.calc_CNF(bit), -spec_identifier_THEN_B_1.calc_CNF(bit), equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_1.calc_CNF(bit), spec_identifier_THEN_B_1.calc_CNF(bit), -equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_THEN_A_1.calc_CNF(bit), -spec_identifier_THEN_B_1.calc_CNF(bit), -equal_1_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    	    
	}

	for (int bit = 0; bit < N_THEN_2_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_2.calc_CNF(bit), -spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), -equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_THEN_A_2.calc_CNF(bit), -spec_identifier_THEN_B_2.calc_CNF(bit), -equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    
	}
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &sUNUSED(aux_Variable_cnt),
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              sUNUSED(state_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              sUNUSED(state_THEN_B_1),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 int                              sUNUSED(state_THEN_B_2)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	total_Literal_cnt += N_THEN_1_Bits * N_IF_Bits + N_THEN_1_Bits + N_THEN_2_Bits * N_IF_Bits + N_THEN_2_Bits;

	return (N_THEN_1_Bits + N_THEN_2_Bits);
    }
	

    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              state_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    int                              state_THEN_B_2,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              state_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              state_THEN_B_1,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 int                              state_THEN_B_2,
							 int                              sUNUSED(weight))
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    	    
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
		    
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {		
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back( -spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    
	}
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              sUNUSED(state_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              sUNUSED(state_THEN_B_1),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_2)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_THEN_2_Bits;
	total_Literal_cnt += N_THEN_1_Bits * N_IF_Bits + N_THEN_1_Bits + 12 * N_THEN_2_Bits;

	return (N_THEN_1_Bits + 5 * N_THEN_2_Bits);
    }
	

    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              state_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_2.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_2_auxiliary(m_variable_store, "eql_2_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_2_Bits - 1));
	m_auxiliary_Identifiers[equal_2_auxiliary.get_First_CNFVariable()] = equal_2_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_2_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_2_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_THEN_2_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A_2.calc_String(bit).c_str(), spec_identifier_THEN_B_2.calc_String(bit).c_str(), equal_2_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              state_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              state_THEN_B_1,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
							 int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_2.get_StateIdentifier()->get_Log2_StateCount())

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_2_auxiliary(m_variable_store, "eql_2_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_2_Bits - 1));
	m_auxiliary_Identifiers[equal_2_auxiliary.get_First_CNFVariable()] = equal_2_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {		
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    	    	    
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    Literals.push_back(equal_2_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
	    
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 1;
	    }
#endif	    	    
	}

	for (int bit = 0; bit < N_THEN_2_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_2.calc_CNF(bit), -spec_identifier_THEN_B_2.calc_CNF(bit), equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_2.calc_CNF(bit), spec_identifier_THEN_B_2.calc_CNF(bit), -equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_2.calc_CNF(bit), -spec_identifier_THEN_B_2.calc_CNF(bit), -equal_2_auxiliary.calc_CNF(sIntegerIndex(bit)));
#ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 4;
	    }
#endif	    
	}
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              sUNUSED(state_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              sUNUSED(state_THEN_B_1),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 int                              sUNUSED(state_THEN_B_2),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_3)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_THEN_3_Bits;
	total_Literal_cnt +=   N_THEN_1_Bits * N_IF_Bits + N_THEN_1_Bits
	                     + N_THEN_2_Bits * N_IF_Bits + N_THEN_2_Bits
	                     + N_THEN_3_Bits * N_IF_Bits + N_THEN_3_Bits
	                     + 12 * N_THEN_3_Bits;

	return (N_THEN_1_Bits + N_THEN_2_Bits + 5 * N_THEN_3_Bits);
    }
	

    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              state_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    int                              state_THEN_B_2,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_3,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_3.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_3_auxiliary(m_variable_store, "eql_3_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_3_Bits - 1));
	m_auxiliary_Identifiers[equal_3_auxiliary.get_First_CNFVariable()] = equal_3_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_3_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_3_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_THEN_3_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A_3.calc_String(bit).c_str(), spec_identifier_THEN_B_3.calc_String(bit).c_str(), equal_3_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A_3.calc_String(bit).c_str(), spec_identifier_THEN_B_3.calc_String(bit).c_str(), equal_3_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A_3.calc_String(bit).c_str(), spec_identifier_THEN_B_3.calc_String(bit).c_str(), equal_3_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A_3.calc_String(bit).c_str(), spec_identifier_THEN_B_3.calc_String(bit).c_str(), equal_3_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A_3.calc_CNF(bit), spec_identifier_THEN_B_3.calc_CNF(bit), equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A_3.calc_CNF(bit), spec_identifier_THEN_B_3.calc_CNF(bit), equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A_3.calc_CNF(bit), spec_identifier_THEN_B_3.calc_CNF(bit), equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A_3.calc_CNF(bit), spec_identifier_THEN_B_3.calc_CNF(bit), equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              state_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              state_THEN_B_1,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 int                              state_THEN_B_2,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_3,
							 int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_3.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_3_auxiliary(m_variable_store, "eql_3_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_3_Bits - 1));
	m_auxiliary_Identifiers[equal_3_auxiliary.get_First_CNFVariable()] = equal_3_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    Literals.push_back(equal_3_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_THEN_3_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A_3.calc_CNF(bit), spec_identifier_THEN_B_3.calc_CNF(bit), equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_3.calc_CNF(bit), -spec_identifier_THEN_B_3.calc_CNF(bit), equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_3.calc_CNF(bit), spec_identifier_THEN_B_3.calc_CNF(bit), -equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_THEN_A_3.calc_CNF(bit), -spec_identifier_THEN_B_3.calc_CNF(bit), -equal_3_auxiliary.calc_CNF(sIntegerIndex(bit)));
		
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }    


    int sStateClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							 int                             &total_Literal_cnt,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              sUNUSED(state_IF_B),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              sUNUSED(state_THEN_B_1),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 int                              sUNUSED(state_THEN_B_2),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							 int                              sUNUSED(state_THEN_B_3),
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
							 const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_4)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_4_Bits = spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_THEN_4_Bits;
	total_Literal_cnt +=   N_THEN_1_Bits * N_IF_Bits + N_THEN_1_Bits
	                     + N_THEN_2_Bits * N_IF_Bits + N_THEN_2_Bits
	                     + N_THEN_3_Bits * N_IF_Bits + N_THEN_3_Bits
	                     + N_THEN_4_Bits * N_IF_Bits + N_THEN_4_Bits
	                     + 12 * N_THEN_4_Bits;

	return (N_THEN_1_Bits + N_THEN_2_Bits + N_THEN_3_Bits + 5 * N_THEN_4_Bits);
    }
	

    int sStateClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              state_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    int                              state_THEN_B_2,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							    int                              state_THEN_B_3,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_4,
							    bool                             string,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_4.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_4_Bits = spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_4_auxiliary(m_variable_store, "eql_4_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_4_Bits - 1));
	m_auxiliary_Identifiers[equal_4_auxiliary.get_First_CNFVariable()] = equal_4_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_3) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier_THEN_A_3.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier_THEN_A_3.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier_THEN_A_3.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier_THEN_A_3.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_4_Bits; ++bit_1)
	{
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_2).c_str());
		    }
		    else
		    {
			fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_2));
		    }
		}
	    }
	    if (string)
	    {
		fprintf(fw, "%s 0\n", equal_4_auxiliary.calc_String(sIntegerIndex(bit_1)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d 0\n", equal_4_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    }
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_THEN_4_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s %s %s 0\n", spec_identifier_THEN_A_4.calc_String(bit).c_str(), spec_identifier_THEN_B_4.calc_String(bit).c_str(), equal_4_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_THEN_A_4.calc_String(bit).c_str(), spec_identifier_THEN_B_4.calc_String(bit).c_str(), equal_4_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_THEN_A_4.calc_String(bit).c_str(), spec_identifier_THEN_B_4.calc_String(bit).c_str(), equal_4_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		fprintf(fw, "%s -%s -%s 0\n", spec_identifier_THEN_A_4.calc_String(bit).c_str(), spec_identifier_THEN_B_4.calc_String(bit).c_str(), equal_4_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d %d %d 0\n", spec_identifier_THEN_A_4.calc_CNF(bit), spec_identifier_THEN_B_4.calc_CNF(bit), equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_THEN_A_4.calc_CNF(bit), spec_identifier_THEN_B_4.calc_CNF(bit), equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_THEN_A_4.calc_CNF(bit), spec_identifier_THEN_B_4.calc_CNF(bit), equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
		fprintf(fw, "%d -%d -%d 0\n", spec_identifier_THEN_A_4.calc_CNF(bit), spec_identifier_THEN_B_4.calc_CNF(bit), equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							 int                              state_IF_B,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							 int                              state_THEN_B_1,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							 int                              state_THEN_B_2,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							 int                              state_THEN_B_3,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
							 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_4,
							 int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_4.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_4_Bits = spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier equal_4_auxiliary(m_variable_store, "eql_4_aux-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_THEN_4_Bits - 1));
	m_auxiliary_Identifiers[equal_4_auxiliary.get_First_CNFVariable()] = equal_4_auxiliary;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    if (calc_Bit(bit_1, state_THEN_B_3) == 1)
	    {
		Literals.push_back(spec_identifier_THEN_A_3.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(-spec_identifier_THEN_A_3.calc_CNF(bit_1));
	    }
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_4_Bits; ++bit_1)
	{
	    std::vector<int> Literals;
	    for (int bit_2 = 0; bit_2 < N_IF_Bits; ++bit_2)
	    {
		if (calc_Bit(bit_2, state_IF_B) == 1)
		{
		    Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_2));
		}
		else
		{
		    Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_2));
		}
	    }
	    Literals.push_back(equal_4_auxiliary.calc_CNF(sIntegerIndex(bit_1)));
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;
	}

	for (int bit = 0; bit < N_THEN_4_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_THEN_A_4.calc_CNF(bit), spec_identifier_THEN_B_4.calc_CNF(bit), equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_4.calc_CNF(bit), -spec_identifier_THEN_B_4.calc_CNF(bit), equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, -spec_identifier_THEN_A_4.calc_CNF(bit), spec_identifier_THEN_B_4.calc_CNF(bit), -equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    cast_Clause(solver, spec_identifier_THEN_A_4.calc_CNF(bit), -spec_identifier_THEN_B_4.calc_CNF(bit), -equal_4_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    Clause_cnt += 4;
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }    


    int sStateClauseGenerator::count_DifferenceConstraint(int                              &aux_Variable_cnt,
							  int                              &total_Literal_cnt,
							  const sSpecifiedStateIdentifier  &spec_identifier_A,
							  SpecifiedStateIdentifiers_vector &spec_Identifiers_B) const
    {
	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_Identifiers = spec_Identifiers_B.size();
	aux_Variable_cnt += N_Identifiers * N_Bits;

	total_Literal_cnt += 7 * N_Identifiers * N_Bits;

	return N_Identifiers * (1 + 2 * N_Bits);
    }
	

    int sStateClauseGenerator::generate_DifferenceConstraint(FILE                             *fw,
							     const sSpecifiedStateIdentifier  &spec_identifier_A,
							     SpecifiedStateIdentifiers_vector &spec_Identifiers_B,
							     bool                              string,
							     int                               sUNUSED(weight))
    {
	#ifdef sDEBUG
	{
	    for (SpecifiedStateIdentifiers_vector::const_iterator identifier_B = spec_Identifiers_B.begin(); identifier_B != spec_Identifiers_B.end(); ++identifier_B)
	    {
		sASSERT(spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount() == identifier_B->get_StateIdentifier()->get_Log2_StateCount());
	    }
	}
	#endif

	int Clause_cnt = 0;
	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	for (SpecifiedStateIdentifiers_vector::const_iterator identifier_B = spec_Identifiers_B.begin(); identifier_B != spec_Identifiers_B.end(); ++identifier_B)
	{
	    sIndexableIdentifier diff_auxiliary(m_variable_store, "diff_aux_" +  sInt_32_to_String(identifier_B - spec_Identifiers_B.begin()) + "-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Bits - 1));
	    m_auxiliary_Identifiers[diff_auxiliary.get_First_CNFVariable()] = diff_auxiliary;

	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (string)
		{
		    fprintf(fw, "%s ", diff_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", diff_auxiliary.calc_CNF(sIntegerIndex(bit)));
		}
	    }
	    fprintf(fw, "0\n");
	    ++Clause_cnt;

	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_A.calc_String(bit).c_str(), identifier_B->calc_String(bit).c_str(), diff_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		    fprintf(fw, "%s %s -%s 0\n", spec_identifier_A.calc_String(bit).c_str(), identifier_B->calc_String(bit).c_str(), diff_auxiliary.calc_String(sIntegerIndex(bit)).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_A.calc_CNF(bit), identifier_B->calc_CNF(bit), diff_auxiliary.calc_CNF(sIntegerIndex(bit)));
		    fprintf(fw, "%d %d -%d 0\n", spec_identifier_A.calc_CNF(bit), identifier_B->calc_CNF(bit), diff_auxiliary.calc_CNF(sIntegerIndex(bit)));
		}
	    }
	    Clause_cnt += 2;
	}

	return Clause_cnt;
    }



    void sStateClauseGenerator::cast_DifferenceConstraint(Glucose::Solver                  *solver,
							  const sSpecifiedStateIdentifier  &spec_identifier_A,
							  SpecifiedStateIdentifiers_vector &spec_Identifiers_B,
							  int                               sUNUSED(weight))
    {
	#ifdef sDEBUG
	{
	    for (SpecifiedStateIdentifiers_vector::const_iterator identifier_B = spec_Identifiers_B.begin(); identifier_B != spec_Identifiers_B.end(); ++identifier_B)
	    {
		sASSERT(spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount() == identifier_B->get_StateIdentifier()->get_Log2_StateCount());
	    }
	}
	#endif

	int Clause_cnt = 0;
	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	for (SpecifiedStateIdentifiers_vector::const_iterator identifier_B = spec_Identifiers_B.begin(); identifier_B != spec_Identifiers_B.end(); ++identifier_B)
	{
	    sIndexableIdentifier diff_auxiliary(m_variable_store, "diff_aux_" +  sInt_32_to_String(identifier_B - spec_Identifiers_B.begin()) + "-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Bits - 1));
	    m_auxiliary_Identifiers[diff_auxiliary.get_First_CNFVariable()] = diff_auxiliary;

	    std::vector<int> Literals;
	    
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		{
		    Literals.push_back(diff_auxiliary.calc_CNF(sIntegerIndex(bit)));
		}
	    }
	    cast_Clause(solver, Literals);
	    ++Clause_cnt;

	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		cast_Clause(solver, -spec_identifier_A.calc_CNF(bit), -identifier_B->calc_CNF(bit), -diff_auxiliary.calc_CNF(sIntegerIndex(bit)));
		cast_Clause(solver, spec_identifier_A.calc_CNF(bit), identifier_B->calc_CNF(bit), -diff_auxiliary.calc_CNF(sIntegerIndex(bit)));
	    }
	}
    }    


    int sStateClauseGenerator::count_AllDifferenceConstraint(int                              &aux_Variable_cnt,
							     int                              &total_Literal_cnt,
							     SpecifiedStateIdentifiers_vector &spec_Identifiers) const
    {
	int N_Bits = spec_Identifiers[0].get_StateIdentifier()->get_Log2_StateCount();
	int N_Identifiers = spec_Identifiers.size();
	aux_Variable_cnt += N_Identifiers * (N_Identifiers - 1) * N_Bits;

//	7 * N_Identifiers * N_Bits;
	total_Literal_cnt += 7 * N_Identifiers * (N_Identifiers - 1) * N_Bits;

	return N_Identifiers * (N_Identifiers - 1) * (1 + 2 * N_Bits);
    }


    int sStateClauseGenerator::generate_AllDifferenceConstraint(FILE                             *fw,
								SpecifiedStateIdentifiers_vector &spec_Identifiers,
								bool                              string,
								int                               weight)
    {
	int Clause_cnt = 0;

	for (SpecifiedStateIdentifiers_vector::const_iterator identifier = spec_Identifiers.begin(); identifier != spec_Identifiers.end(); ++identifier)
	{
	    SpecifiedStateIdentifiers_vector diff_spec_Identifiers;
	 
	    SpecifiedStateIdentifiers_vector::const_iterator diff_identifier;
	    for (diff_identifier = spec_Identifiers.begin(); diff_identifier != identifier; ++diff_identifier)
	    {
		diff_spec_Identifiers.push_back(*diff_identifier);
	    }
	    for (++diff_identifier; diff_identifier != spec_Identifiers.end(); ++diff_identifier)
	    {
		diff_spec_Identifiers.push_back(*diff_identifier);
	    }	   
	    Clause_cnt += generate_DifferenceConstraint(fw, *identifier, diff_spec_Identifiers, string, weight);
	}

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_AllDifferenceConstraint(Glucose::Solver                  *solver,
							     SpecifiedStateIdentifiers_vector &spec_Identifiers,
							     int                               weight)
    {
	for (SpecifiedStateIdentifiers_vector::const_iterator identifier = spec_Identifiers.begin(); identifier != spec_Identifiers.end(); ++identifier)
	{
	    SpecifiedStateIdentifiers_vector diff_spec_Identifiers;
	 
	    SpecifiedStateIdentifiers_vector::const_iterator diff_identifier;
	    for (diff_identifier = spec_Identifiers.begin(); diff_identifier != identifier; ++diff_identifier)
	    {
		diff_spec_Identifiers.push_back(*diff_identifier);
	    }
	    for (++diff_identifier; diff_identifier != spec_Identifiers.end(); ++diff_identifier)
	    {
		diff_spec_Identifiers.push_back(*diff_identifier);
	    }	   
	    cast_DifferenceConstraint(solver, *identifier, diff_spec_Identifiers, weight);
	}
    }    


    int sStateClauseGenerator::count_CaseSplitting(int                              &aux_Variable_cnt,
						   int                              &total_Literal_cnt,
						   SpecifiedStateIdentifiers_vector &split_Identifiers,
						   States_vector                    &sUNUSED(split_States)) const
    {
	int N_Splits = split_Identifiers.size();
	SpecifiedStateIdentifiers_vector::const_iterator split_identifier = split_Identifiers.begin();
	int N_Bits = split_identifier->get_StateIdentifier()->get_Log2_StateCount();

	#ifdef sDEBUG
	{
	    SpecifiedStateIdentifiers_vector::const_iterator split_identifier = split_Identifiers.begin();
	    int N_Bits = split_identifier->get_StateIdentifier()->get_Log2_StateCount();

	    for (++split_identifier; split_identifier != split_Identifiers.end(); ++split_identifier)
	    {
		sASSERT(N_Bits == split_identifier->get_StateIdentifier()->get_Log2_StateCount());
	    }
	}
	#endif

	aux_Variable_cnt += N_Splits;
	total_Literal_cnt += 1 + N_Bits + 2 * N_Splits * N_Bits + N_Splits;

	return 2 + (N_Splits - 1) * N_Bits;
    }


    int sStateClauseGenerator::generate_CaseSplitting(FILE                             *fw,
						      SpecifiedStateIdentifiers_vector &split_Identifiers,
						      States_vector                    &split_States,
						      bool                              string,
						      int                               sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Splits = split_Identifiers.size();
	
	sIndexableIdentifier split_auxiliary(m_variable_store, "split_aux_" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Splits - 1));
	m_auxiliary_Identifiers[split_auxiliary.get_First_CNFVariable()] = split_auxiliary;

	SpecifiedStateIdentifiers_vector::const_iterator split_identifier = split_Identifiers.begin();
	int N_Bits = split_identifier->get_StateIdentifier()->get_Log2_StateCount();

	if (string)
	{
	    fprintf(fw, "-%s ", split_auxiliary.calc_String(sIntegerIndex(0)).c_str());
	}
	else
	{
	    fprintf(fw, "-%d ", split_auxiliary.calc_CNF(sIntegerIndex(0)));
	}
	
	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    if (calc_Bit(bit, split_States[0]) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", split_identifier->calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", split_identifier->calc_CNF(bit));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", split_identifier->calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", split_identifier->calc_CNF(bit));
		}
	    }
	}

	fprintf(fw, "0\n");
	++Clause_cnt;

	for (++split_identifier; split_identifier != split_Identifiers.end(); ++split_identifier)
	{
	    int N_Bits = split_identifier->get_StateIdentifier()->get_Log2_StateCount();
	    	    
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (calc_Bit(bit, split_States[split_identifier - split_Identifiers.begin()]) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s %s 0\n", split_auxiliary.calc_String(sIntegerIndex(split_identifier - split_Identifiers.begin())).c_str(), split_identifier->calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d %d 0\n", split_auxiliary.calc_CNF(sIntegerIndex(split_identifier - split_Identifiers.begin())), split_identifier->calc_CNF(bit));
		    }
		    ++Clause_cnt;
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "-%s -%s 0\n", split_auxiliary.calc_String(sIntegerIndex(split_identifier - split_Identifiers.begin())).c_str(), split_identifier->calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d -%d 0\n", split_auxiliary.calc_CNF(sIntegerIndex(split_identifier - split_Identifiers.begin())), split_identifier->calc_CNF(bit));
		    }
		    ++Clause_cnt;
		}
	    }
	}

	for (int split = 0; split < N_Splits; ++split)
	{
	    if (string)
	    {
		fprintf(fw, "%s ", split_auxiliary.calc_String(sIntegerIndex(split)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d ", split_auxiliary.calc_CNF(sIntegerIndex(split)));
	    }
	}

	fprintf(fw, "0\n");
	++Clause_cnt;

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_CaseSplitting(Glucose::Solver                  *solver,
						   SpecifiedStateIdentifiers_vector &split_Identifiers,
						   States_vector                    &split_States,
						   int                               sUNUSED(weight))
    {
	int N_Splits = split_Identifiers.size();
	
	sIndexableIdentifier split_auxiliary(m_variable_store, "split_aux_" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Splits - 1));
	m_auxiliary_Identifiers[split_auxiliary.get_First_CNFVariable()] = split_auxiliary;

	SpecifiedStateIdentifiers_vector::const_iterator split_identifier = split_Identifiers.begin();
	int N_Bits = split_identifier->get_StateIdentifier()->get_Log2_StateCount();

	std::vector<int> Literals;
	Literals.push_back(-split_auxiliary.calc_CNF(sIntegerIndex(0)));
	
	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    if (calc_Bit(bit, split_States[0]) == 1)
	    {
		Literals.push_back(-split_identifier->calc_CNF(bit));
	    }
	    else
	    {
		Literals.push_back(split_identifier->calc_CNF(bit));
	    }
	}
	cast_Clause(solver, Literals);

	for (++split_identifier; split_identifier != split_Identifiers.end(); ++split_identifier)
	{
	    int N_Bits = split_identifier->get_StateIdentifier()->get_Log2_StateCount();

	    std::vector<int> Literals;
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (calc_Bit(bit, split_States[split_identifier - split_Identifiers.begin()]) == 1)
		{
		    cast_Clause(solver, -split_auxiliary.calc_CNF(sIntegerIndex(split_identifier - split_Identifiers.begin())), split_identifier->calc_CNF(bit));
		}
		else
		{
		    cast_Clause(solver, -split_auxiliary.calc_CNF(sIntegerIndex(split_identifier - split_Identifiers.begin())), -split_identifier->calc_CNF(bit));
		}
	    }
	}

	{
	    std::vector<int> Literals;
	    for (int split = 0; split < N_Splits; ++split)
	    {
		Literals.push_back(split_auxiliary.calc_CNF(sIntegerIndex(split)));
	    }
	    cast_Clause(solver, Literals);
	}
    }    


    int sStateClauseGenerator::count_DisjunctiveEquality(int                              &aux_Variable_cnt,
							 int                              &total_Literal_cnt,
							 SpecifiedStateIdentifiers_vector &disj_Identifiers,
							 States_vector                    &sUNUSED(disj_States)) const
    {
	int N_Disjs = disj_Identifiers.size();
	SpecifiedStateIdentifiers_vector::const_iterator disj_identifier = disj_Identifiers.begin();
	int N_Bits = disj_identifier->get_StateIdentifier()->get_Log2_StateCount();

	#ifdef sDEBUG
	{
	    SpecifiedStateIdentifiers_vector::const_iterator disj_identifier = disj_Identifiers.begin();
	    int N_Bits = disj_identifier->get_StateIdentifier()->get_Log2_StateCount();

	    for (++disj_identifier; disj_identifier != disj_Identifiers.end(); ++disj_identifier)
	    {
		sASSERT(N_Bits == disj_identifier->get_StateIdentifier()->get_Log2_StateCount());
	    }
	}
	#endif

	aux_Variable_cnt += N_Disjs;
	total_Literal_cnt += 2 * N_Disjs * N_Bits + N_Disjs;

	return 1 + N_Disjs * N_Bits;
    }

	
    int sStateClauseGenerator::generate_DisjunctiveEquality(FILE                             *fw,
							    SpecifiedStateIdentifiers_vector &disj_Identifiers,
							    States_vector                    &disj_States,
							    bool                              string,
							    int                               sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Disjs = disj_Identifiers.size();
	
	sIndexableIdentifier disj_auxiliary(m_variable_store, "disj_aux_" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Disjs - 1));
	m_auxiliary_Identifiers[disj_auxiliary.get_First_CNFVariable()] = disj_auxiliary;

	SpecifiedStateIdentifiers_vector::const_iterator disj_identifier = disj_Identifiers.begin();
	for (; disj_identifier != disj_Identifiers.end(); ++disj_identifier)
	{
	    int N_Bits = disj_identifier->get_StateIdentifier()->get_Log2_StateCount();
	    	    
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (calc_Bit(bit, disj_States[disj_identifier - disj_Identifiers.begin()]) == 1)
		{
		    if (string)
		    {
			fprintf(fw, "-%s %s 0\n", disj_auxiliary.calc_String(sIntegerIndex(disj_identifier - disj_Identifiers.begin())).c_str(), disj_identifier->calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d %d 0\n", disj_auxiliary.calc_CNF(sIntegerIndex(disj_identifier - disj_Identifiers.begin())), disj_identifier->calc_CNF(bit));
		    }
		    ++Clause_cnt;
		}
		else
		{
		    if (string)
		    {
			fprintf(fw, "-%s -%s 0\n", disj_auxiliary.calc_String(sIntegerIndex(disj_identifier - disj_Identifiers.begin())).c_str(), disj_identifier->calc_String(bit).c_str());
		    }
		    else
		    {
			fprintf(fw, "-%d -%d 0\n", disj_auxiliary.calc_CNF(sIntegerIndex(disj_identifier - disj_Identifiers.begin())), disj_identifier->calc_CNF(bit));
		    }
		    ++Clause_cnt;
		}
	    }
	}
	for (int disj = 0; disj < N_Disjs; ++disj)
	{
	    if (string)
	    {
		fprintf(fw, "%s ", disj_auxiliary.calc_String(sIntegerIndex(disj)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d ", disj_auxiliary.calc_CNF(sIntegerIndex(disj)));
	    }
	}

	fprintf(fw, "0\n");
	++Clause_cnt;

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_DisjunctiveEquality(Glucose::Solver                  *solver,
							 SpecifiedStateIdentifiers_vector &disj_Identifiers,
							 States_vector                    &disj_States,
							 int                               sUNUSED(weight))
    {
	int N_Disjs = disj_Identifiers.size();
	
	sIndexableIdentifier disj_auxiliary(m_variable_store, "disj_aux_" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Disjs - 1));
	m_auxiliary_Identifiers[disj_auxiliary.get_First_CNFVariable()] = disj_auxiliary;

	SpecifiedStateIdentifiers_vector::const_iterator disj_identifier = disj_Identifiers.begin();
	for (; disj_identifier != disj_Identifiers.end(); ++disj_identifier)
	{
	    int N_Bits = disj_identifier->get_StateIdentifier()->get_Log2_StateCount();
	    	    
	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		if (calc_Bit(bit, disj_States[disj_identifier - disj_Identifiers.begin()]) == 1)
		{
		    cast_Clause(solver, -disj_auxiliary.calc_CNF(sIntegerIndex(disj_identifier - disj_Identifiers.begin())), disj_identifier->calc_CNF(bit));
		}
		else
		{
		    cast_Clause(solver, -disj_auxiliary.calc_CNF(sIntegerIndex(disj_identifier - disj_Identifiers.begin())), -disj_identifier->calc_CNF(bit));
		}
	    }
	}
	std::vector<int> Literals;
	for (int disj = 0; disj < N_Disjs; ++disj)
	{
	    Literals.push_back(disj_auxiliary.calc_CNF(sIntegerIndex(disj)));
	}
	cast_Clause(solver, Literals);
    }    


    int sStateClauseGenerator::count_LargeConditionalEquality(int                              &sUNUSED(aux_Variable_cnt),
							      int                              &sUNUSED(total_Literal_cnt),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_IF_A),
							      int                               sUNUSED(state_IF_B),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_A),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B))
    {
	sASSERT(false);	
	return 0;
    }

    
    int sStateClauseGenerator::generate_LargeConditionalEquality(FILE                             *sUNUSED(fw),
								 const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_IF_A),
								 int                               sUNUSED(state_IF_B),
								 SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_A),
								 SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B),
								 bool                              sUNUSED(string),
								 int                               sUNUSED(sUNUSED(weight)))
    {
	sASSERT(false);
	return 0;
    }


    void sStateClauseGenerator::cast_LargeConditionalEquality(Glucose::Solver                  *sUNUSED(solver),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_IF_A),
							      int                               sUNUSED(state_IF_B),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_A),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B),
							      int                               sUNUSED(sUNUSED(weight)))
    {
	sASSERT(false);
    }    
    

    int sStateClauseGenerator::count_LargeConditionalEquality(int                              &sUNUSED(aux_Variable_cnt),
							      int                              &sUNUSED(total_Literal_cnt),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_IF_A),
							      int                               sUNUSED(state_IF_B),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_THEN_A_1),
							      int                               sUNUSED(state_THEN_B_1),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_THEN_A_2),
							      int                               sUNUSED(state_THEN_B_2),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_A_3),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B_3))
    {
	sASSERT(false);
	return 0;
    }
    

    int sStateClauseGenerator::generate_LargeConditionalEquality(FILE                             *sUNUSED(fw),
								 const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_IF_A),
								 int                               sUNUSED(state_IF_B),
								 const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_THEN_A_1),
								 int                               sUNUSED(state_THEN_B_1),
								 const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_THEN_A_2),
								 int                               sUNUSED(state_THEN_B_2),
								 SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_A_3),
								 SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B_3),
								 bool                              sUNUSED(string),
								 int                               sUNUSED(sUNUSED(weight)))
    {
	sASSERT(false);
	return 0;
    }

    
    void sStateClauseGenerator::cast_LargeConditionalEquality(Glucose::Solver                  *sUNUSED(solver),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_IF_A),
							      int                               sUNUSED(state_IF_B),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_THEN_A_1),
							      int                               sUNUSED(state_THEN_B_1),
							      const sSpecifiedStateIdentifier  &sUNUSED(spec_identifier_THEN_A_2),
							      int                               sUNUSED(state_THEN_B_2),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_A_3),
							      SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B_3),
							      int                               sUNUSED(sUNUSED(weight)))
    {
	sASSERT(false);
    }    


    sSpecifiedIdentifier sStateClauseGenerator::translate_AuxiliaryCNFVariable(int cnf_variable) const
    {
	AuxiliaryIdentifiers_map::const_iterator auxiliary_identifier = m_auxiliary_Identifiers.upper_bound(cnf_variable);

	/*
	for (AuxiliaryIdentifiers_map::const_iterator identifier = m_auxiliary_Identifiers.begin(); identifier != m_auxiliary_Identifiers.end(); ++identifier)
	{
	    printf("First var:%d\n", identifier->first);
	    identifier->second.to_Screen();
	}
	//	exit(0);
	*/
       
	if (auxiliary_identifier != m_auxiliary_Identifiers.end())
	{
	    if (auxiliary_identifier->first > cnf_variable)
	    {
		--auxiliary_identifier;
	    }
	    if (auxiliary_identifier->second.is_Anonymous())
	    {
		return sSpecifiedIdentifier();
	    }
	    else
	    {
		return auxiliary_identifier->second.translate_CNFVariable(cnf_variable);
	    }
	}

	return sSpecifiedIdentifier();
    }

    int sStateClauseGenerator::count_LEXLess_Constraint(int                             &aux_Variable_cnt,
							int                             &total_Literal_cnt,
							const sSpecifiedStateIdentifier &spec_identifier_A,
							const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_B)) const
    {
	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += N_Bits;
	total_Literal_cnt += 3 * (N_Bits * (N_Bits + 1)) + 2 * N_Bits + N_Bits;

	return (1 + (N_Bits * (N_Bits + 1)));
    }

	
    int sStateClauseGenerator::generate_LEXLess_Constraint(FILE                            *fw,
							   const sSpecifiedStateIdentifier &spec_identifier_A,
							   const sSpecifiedStateIdentifier &spec_identifier_B,
							   bool                             string,
							   int                              sUNUSED(sUNUSED(weight)))
    {
	int Clause_cnt = 0;

	sASSERT(spec_identifier_A.get_StateIdentifier()->get_StateCount() == spec_identifier_B.get_StateIdentifier()->get_StateCount());

	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier lex_less_auxiliary(m_variable_store, "lex_less-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Bits - 1));
	m_auxiliary_Identifiers[lex_less_auxiliary.get_First_CNFVariable()] = lex_less_auxiliary;

	for (int diff_pos = N_Bits - 1; diff_pos >= 0; --diff_pos)
	{
	    for (int equal_bit = N_Bits - 1; equal_bit > diff_pos; --equal_bit)
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s %s 0\n", lex_less_auxiliary.calc_String(sIntegerIndex(diff_pos)).c_str(), spec_identifier_A.calc_String(equal_bit).c_str(), spec_identifier_B.calc_String(equal_bit).c_str());
		    fprintf(fw, "-%s %s -%s 0\n", lex_less_auxiliary.calc_String(sIntegerIndex(diff_pos)).c_str(), spec_identifier_A.calc_String(equal_bit).c_str(), spec_identifier_B.calc_String(equal_bit).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d %d 0\n", lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), spec_identifier_A.calc_CNF(equal_bit), spec_identifier_B.calc_CNF(equal_bit));
		    fprintf(fw, "-%d %d -%d 0\n", lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), spec_identifier_A.calc_CNF(equal_bit), spec_identifier_B.calc_CNF(equal_bit));
		}
		Clause_cnt += 2;
	    }
	    if (string)
	    {
		fprintf(fw, "-%s -%s 0\n", lex_less_auxiliary.calc_String(sIntegerIndex(diff_pos)).c_str(), spec_identifier_A.calc_String(diff_pos).c_str());
		fprintf(fw, "-%s %s 0\n", lex_less_auxiliary.calc_String(sIntegerIndex(diff_pos)).c_str(), spec_identifier_B.calc_String(diff_pos).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d 0\n", lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), spec_identifier_A.calc_CNF(diff_pos));
		fprintf(fw, "-%d %d 0\n", lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), spec_identifier_B.calc_CNF(diff_pos));
	    }
	    Clause_cnt += 2;
	}

	for (int diff_pos = N_Bits - 1; diff_pos >= 0; --diff_pos)
	{
	    if (string)
	    {
		fprintf(fw, "%s ", lex_less_auxiliary.calc_String(sIntegerIndex(diff_pos)).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d ", lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)));
	    }
	}
	fprintf(fw, "0\n");

	++Clause_cnt;

	return Clause_cnt;
    }


    void sStateClauseGenerator::cast_LEXLess_Constraint(Glucose::Solver                 *solver,
							const sSpecifiedStateIdentifier &spec_identifier_A,
							const sSpecifiedStateIdentifier &spec_identifier_B,
							int                              sUNUSED(sUNUSED(weight)))
    {
	sASSERT(spec_identifier_A.get_StateIdentifier()->get_StateCount() == spec_identifier_B.get_StateIdentifier()->get_StateCount());

	int N_Bits = spec_identifier_A.get_StateIdentifier()->get_Log2_StateCount();

	sIndexableIdentifier lex_less_auxiliary(m_variable_store, "lex_less-" + sInt_32_to_String(m_aux_Identifier_cnt++), sIntegerScope(0, N_Bits - 1));
	m_auxiliary_Identifiers[lex_less_auxiliary.get_First_CNFVariable()] = lex_less_auxiliary;

	for (int diff_pos = N_Bits - 1; diff_pos >= 0; --diff_pos)
	{
	    for (int equal_bit = N_Bits - 1; equal_bit > diff_pos; --equal_bit)
	    {
		cast_Clause(solver, -lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), -spec_identifier_A.calc_CNF(equal_bit), spec_identifier_B.calc_CNF(equal_bit));
		cast_Clause(solver, -lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), spec_identifier_A.calc_CNF(equal_bit), -spec_identifier_B.calc_CNF(equal_bit));
	    }
	    cast_Clause(solver, -lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), -spec_identifier_A.calc_CNF(diff_pos));
	    cast_Clause(solver, -lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)), spec_identifier_B.calc_CNF(diff_pos));
	}

	std::vector<int> Literals;
	for (int diff_pos = N_Bits - 1; diff_pos >= 0; --diff_pos)
	{
	    Literals.push_back(lex_less_auxiliary.calc_CNF(sIntegerIndex(diff_pos)));
	}
	cast_Clause(solver, Literals);
    }    


/*----------------------------------------------------------------------------*/

    int sStateClauseGenerator::calc_Bit(int bit, int state)
    {
	while (bit-- > 0)
	{
	    state /= 2;
	}
	return state % 2;
    }


/*----------------------------------------------------------------------------*/
    
    void sStateClauseGenerator::cast_Clause(Glucose::Solver *solver, int lit_1)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	
	cast_Clause(solver, Lits);
    }

    
    void sStateClauseGenerator::cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	
	cast_Clause(solver, Lits);
    }

    
    void sStateClauseGenerator::cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2, int lit_3)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	Lits.push_back(lit_3);
		
	cast_Clause(solver, Lits);
    }

    
    void sStateClauseGenerator::cast_Clause(Glucose::Solver *solver, vector<int> &Lits)
    {
	Glucose::vec<Glucose::Lit> glu_Lits;
	
	for (vector<int>::const_iterator lit = Lits.begin(); lit != Lits.end(); ++lit)
	{
	    int glu_var = sABS(*lit) - 1;
	    while (glu_var >= solver->nVars())
	    {
		solver->newVar();
	    }
	    glu_Lits.push((*lit > 0) ? Glucose::mkLit(glu_var) : ~Glucose::mkLit(glu_var));
	}
	solver->addClause(glu_Lits);
    }
     


/*----------------------------------------------------------------------------*/
// sAdvancedClauseGenerator

    sAdvancedClauseGenerator::sAdvancedClauseGenerator(sVariableStore_CNF *variable_store)
	: sStateClauseGenerator(variable_store)
    {
	// nothing
    }


    sAdvancedClauseGenerator::sAdvancedClauseGenerator(const sAdvancedClauseGenerator &clause_generator)
	: sStateClauseGenerator(clause_generator)
    {
	// nothing
    }


    const sAdvancedClauseGenerator& sAdvancedClauseGenerator::operator=(const sAdvancedClauseGenerator &clause_generator)
    {
	sStateClauseGenerator::operator=(clause_generator);
	return *this;
    }


    int sAdvancedClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							    int                             &total_Literal_cnt,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              sUNUSED(state_IF_B),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += 1;
	total_Literal_cnt += N_IF_Bits + 6 * N_THEN_Bits + 1;

	return (1 + 2 * N_THEN_Bits);
    }
	

    int sAdvancedClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							       const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							       int                              state_IF_B,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							       bool                             string,
							       int                              sUNUSED(sUNUSED(weight)))
    {
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	}
	if (string)
	{
	    fprintf(fw, "%s 0\n", "auxiliary");
	}
	else
	{
	    fprintf(fw, "%d 0\n", aux_cnf_variable);
	}
	++Clause_cnt;

	for (int bit_2 = 0; bit_2 < N_THEN_Bits; ++bit_2)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s %s 0\n", "auxiliary", spec_identifier_THEN_A.calc_String(bit_2).c_str(), spec_identifier_THEN_B.calc_String(bit_2).c_str());
		fprintf(fw, "-%s %s -%s 0\n", "auxiliary", spec_identifier_THEN_A.calc_String(bit_2).c_str(), spec_identifier_THEN_B.calc_String(bit_2).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A.calc_CNF(bit_2), spec_identifier_THEN_B.calc_CNF(bit_2));
		fprintf(fw, "-%d %d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A.calc_CNF(bit_2), spec_identifier_THEN_B.calc_CNF(bit_2));
	    }
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sAdvancedClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							    int                              sUNUSED(sUNUSED(weight)))
    {
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	std::vector<int> Literals;
	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	}
	Literals.push_back(aux_cnf_variable);
	cast_Clause(solver, Literals);
	++Clause_cnt;

	for (int bit_2 = 0; bit_2 < N_THEN_Bits; ++bit_2)
	{
	    cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A.calc_CNF(bit_2), spec_identifier_THEN_B.calc_CNF(bit_2));
	    cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A.calc_CNF(bit_2), -spec_identifier_THEN_B.calc_CNF(bit_2));
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }
    

    int sAdvancedClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							    int                             &total_Literal_cnt,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              sUNUSED(state_IF_B),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    int                              sUNUSED(state_THEN_B)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += 1;
	total_Literal_cnt += N_IF_Bits + 2 * N_THEN_Bits + 1;

	return (1 + N_THEN_Bits);
    }
	

    int sAdvancedClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							       const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							       int                              state_IF_B,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							       int                              state_THEN_B,
							       bool                             string,
							       int                              sUNUSED(sUNUSED(weight)))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	}
	if (string)
	{
	    fprintf(fw, "%s 0\n", "auxiliary");
	}
	else
	{
	    fprintf(fw, "%d 0\n", aux_cnf_variable);
	}
	++Clause_cnt;


	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sAdvancedClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							    int                              state_THEN_B,
							    int                              sUNUSED(sUNUSED(weight)))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	std::vector<int> Literals;
	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	}
	Literals.push_back(aux_cnf_variable);
	cast_Clause(solver, Literals);
	++Clause_cnt;


	for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }    


    int sAdvancedClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							    int                             &total_Literal_cnt,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              sUNUSED(state_IF_B),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              sUNUSED(state_THEN_B_1),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_2)) const
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += 1;
	total_Literal_cnt += N_IF_Bits + 2 * N_THEN_1_Bits + 6 * N_THEN_2_Bits + 1;

	return (1 + N_THEN_1_Bits + 2 * N_THEN_2_Bits);
    }
	

    int sAdvancedClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							       const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							       int                              state_IF_B,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							       int                              state_THEN_B_1,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
							       bool                             string,
							       int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_2.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	}
	if (string)
	{
	    fprintf(fw, "%s 0\n", "auxiliary");
	}
	else
	{
	    fprintf(fw, "%d 0\n", aux_cnf_variable);
	}
	++Clause_cnt;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s %s 0\n", "auxiliary", spec_identifier_THEN_A_2.calc_String(bit_1).c_str(), spec_identifier_THEN_B_2.calc_String(bit_1).c_str());
		fprintf(fw, "-%s %s -%s 0\n", "auxiliary", spec_identifier_THEN_A_2.calc_String(bit_1).c_str(), spec_identifier_THEN_B_2.calc_String(bit_1).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1), spec_identifier_THEN_B_2.calc_CNF(bit_1));
		fprintf(fw, "-%d %d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1), spec_identifier_THEN_B_2.calc_CNF(bit_1));
	    }
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sAdvancedClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              state_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_2.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	std::vector<int> Literals;
	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	}
	Literals.push_back(aux_cnf_variable);
	cast_Clause(solver, Literals);
	++Clause_cnt;

	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_2.calc_CNF(bit_1), spec_identifier_THEN_B_2.calc_CNF(bit_1));
	    cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1), -spec_identifier_THEN_B_2.calc_CNF(bit_1));
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }    


    int sAdvancedClauseGenerator::count_ConditionalEquality(int                             &aux_Variable_cnt,
							    int                             &total_Literal_cnt,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              sUNUSED(state_IF_B),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              sUNUSED(state_THEN_B_1),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    int                              sUNUSED(state_THEN_B_2),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							    int                              sUNUSED(state_THEN_B_3),
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
							    const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B_4)) const
    {	
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_4_Bits = spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount();

	aux_Variable_cnt += 1;
	total_Literal_cnt += N_IF_Bits + 2 * N_THEN_1_Bits + 2 * N_THEN_2_Bits + 2 * N_THEN_3_Bits + 6 * N_THEN_4_Bits + 1;

	return (1 + N_THEN_1_Bits + N_THEN_2_Bits + N_THEN_3_Bits + 2 * N_THEN_4_Bits);
    }
	

    int sAdvancedClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							       const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							       int                              state_IF_B,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							       int                              state_THEN_B_1,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							       int                              state_THEN_B_2,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							       int                              state_THEN_B_3,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
							       const sSpecifiedStateIdentifier &spec_identifier_THEN_B_4,
							       bool                             string,
							       int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_4.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_4_Bits = spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	}
	if (string)
	{
	    fprintf(fw, "%s 0\n", "auxiliary");
	}
	else
	{
	    fprintf(fw, "%d 0\n", aux_cnf_variable);
	}
	++Clause_cnt;


	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_3) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A_3.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_3.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A_3.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_3.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_4_Bits; ++bit_1)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s %s 0\n", "auxiliary", spec_identifier_THEN_A_4.calc_String(bit_1).c_str(), spec_identifier_THEN_B_4.calc_String(bit_1).c_str());
		fprintf(fw, "-%s %s -%s 0\n", "auxiliary", spec_identifier_THEN_A_4.calc_String(bit_1).c_str(), spec_identifier_THEN_B_4.calc_String(bit_1).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_4.calc_CNF(bit_1), spec_identifier_THEN_B_4.calc_CNF(bit_1));
		fprintf(fw, "-%d %d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_4.calc_CNF(bit_1), spec_identifier_THEN_B_4.calc_CNF(bit_1));
	    }
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sAdvancedClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
							    const sSpecifiedStateIdentifier &spec_identifier_IF_A,
							    int                              state_IF_B,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
							    int                              state_THEN_B_1,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
							    int                              state_THEN_B_2,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
							    int                              state_THEN_B_3,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
							    const sSpecifiedStateIdentifier &spec_identifier_THEN_B_4,
							    int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B_4.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_3_Bits = spec_identifier_THEN_A_3.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_4_Bits = spec_identifier_THEN_A_4.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	std::vector<int> Literals;
	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	}
	Literals.push_back(aux_cnf_variable);
	cast_Clause(solver, Literals);
	++Clause_cnt;


	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_3) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_3.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_3.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_4_Bits; ++bit_1)
	{
	    cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_4.calc_CNF(bit_1), spec_identifier_THEN_B_4.calc_CNF(bit_1));
	    cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_4.calc_CNF(bit_1), -spec_identifier_THEN_B_4.calc_CNF(bit_1));
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }    


    int sAdvancedClauseGenerator::count_LargeConditionalEquality(int                              &aux_Variable_cnt,
								 int                              &total_Literal_cnt,
								 const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
								 int                               sUNUSED(state_IF_B),
								 SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
								 SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B))
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_Bits = 0;
	int N_Equalities = spec_Identifiers_THEN_A.size();
	
	for (int equality = 0; equality < N_Equalities; ++equality)
	{
	    N_THEN_Bits += spec_Identifiers_THEN_A[equality].get_StateIdentifier()->get_Log2_StateCount();
	}
	aux_Variable_cnt += 1;
	total_Literal_cnt += N_IF_Bits + 6 * N_Equalities * N_THEN_Bits + 1;

	return (1 + 2 * N_THEN_Bits);
    }


    int sAdvancedClauseGenerator::generate_LargeConditionalEquality(FILE                             *fw,
								    const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
								    int                               state_IF_B,
								    SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
								    SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B,
								    bool                              string,
								    int                               sUNUSED(weight))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	}
	if (string)
	{
	    fprintf(fw, "%s 0\n", "auxiliary");
	}
	else
	{
	    fprintf(fw, "%d 0\n", aux_cnf_variable);
	}
	++Clause_cnt;

	int N_Equalities = spec_Identifiers_THEN_A.size();
	sASSERT(spec_Identifiers_THEN_A.size() == spec_Identifiers_THEN_B.size());
	
	for (int equality = 0; equality < N_Equalities; ++equality)
	{
	    sASSERT(spec_Identifiers_THEN_A[equality].get_StateIdentifier()->get_Log2_StateCount() == spec_Identifiers_THEN_B[equality].get_StateIdentifier()->get_Log2_StateCount());
	    int N_THEN_Bits = spec_Identifiers_THEN_A[equality].get_StateIdentifier()->get_Log2_StateCount();
	    
	    for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s %s 0\n", "auxiliary", spec_Identifiers_THEN_A[equality].calc_String(bit_1).c_str(), spec_Identifiers_THEN_B[equality].calc_String(bit_1).c_str());
		    fprintf(fw, "-%s %s -%s 0\n", "auxiliary", spec_Identifiers_THEN_A[equality].calc_String(bit_1).c_str(), spec_Identifiers_THEN_B[equality].calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d %d 0\n", aux_cnf_variable, spec_Identifiers_THEN_A[equality].calc_CNF(bit_1), spec_Identifiers_THEN_B[equality].calc_CNF(bit_1));
		    fprintf(fw, "-%d %d -%d 0\n", aux_cnf_variable, spec_Identifiers_THEN_A[equality].calc_CNF(bit_1), spec_Identifiers_THEN_B[equality].calc_CNF(bit_1));
		}
		Clause_cnt += 2;
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
        #endif

	return Clause_cnt;
    }

    
    void sAdvancedClauseGenerator::cast_LargeConditionalEquality(Glucose::Solver                  *solver,
								 const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
								 int                               state_IF_B,
								 SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
								 SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B,
								 int                               sUNUSED(weight))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	std::vector<int> Literals;
	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	}
	Literals.push_back(aux_cnf_variable);
	cast_Clause(solver, Literals);
	++Clause_cnt;

	int N_Equalities = spec_Identifiers_THEN_A.size();
	sASSERT(spec_Identifiers_THEN_A.size() == spec_Identifiers_THEN_B.size());
	
	for (int equality = 0; equality < N_Equalities; ++equality)
	{
	    sASSERT(spec_Identifiers_THEN_A[equality].get_StateIdentifier()->get_Log2_StateCount() == spec_Identifiers_THEN_B[equality].get_StateIdentifier()->get_Log2_StateCount());
	    int N_THEN_Bits = spec_Identifiers_THEN_A[equality].get_StateIdentifier()->get_Log2_StateCount();
	    
	    for (int bit_1 = 0; bit_1 < N_THEN_Bits; ++bit_1)
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_Identifiers_THEN_A[equality].calc_CNF(bit_1), spec_Identifiers_THEN_B[equality].calc_CNF(bit_1));
		cast_Clause(solver, -aux_cnf_variable, spec_Identifiers_THEN_A[equality].calc_CNF(bit_1), -spec_Identifiers_THEN_B[equality].calc_CNF(bit_1));

		Clause_cnt += 2;
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
        #endif
    }    


    int sAdvancedClauseGenerator::count_LargeConditionalEquality(int                              &aux_Variable_cnt,
								 int                              &total_Literal_cnt,
								 const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
								 int                               sUNUSED(state_IF_B),
								 const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
								 int                               sUNUSED(state_THEN_B_1),
								 const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
								 int                               sUNUSED(state_THEN_B_2),
								 SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
								 SpecifiedStateIdentifiers_vector &sUNUSED(spec_Identifiers_THEN_B_3))
    {
	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	int N_THEN_3_Bits = 0;
	int N_Equalities = spec_Identifiers_THEN_A_3.size();
	
	for (int equality = 0; equality < N_Equalities; ++equality)
	{
	    N_THEN_3_Bits += spec_Identifiers_THEN_A_3[equality].get_StateIdentifier()->get_Log2_StateCount();
	}
	aux_Variable_cnt += 1;
	total_Literal_cnt += N_IF_Bits + 2 * N_THEN_1_Bits + 2 * N_THEN_2_Bits + 6 * N_Equalities * N_THEN_3_Bits + 1;

	return (1 + N_THEN_1_Bits + N_THEN_2_Bits + 2 * N_THEN_3_Bits);
    }

	
    int sAdvancedClauseGenerator::generate_LargeConditionalEquality(FILE                             *fw,
								    const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
								    int                               state_IF_B,
								    const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
								    int                               state_THEN_B_1,
								    const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
								    int                               state_THEN_B_2,
								    SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
								    SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3,
								    bool                              string,
								    int                               sUNUSED(weight))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "%s ", spec_identifier_IF_A.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "%d ", spec_identifier_IF_A.calc_CNF(bit_1));
		}
	    }
	}
	if (string)
	{
	    fprintf(fw, "%s 0\n", "auxiliary");
	}
	else
	{
	    fprintf(fw, "%d 0\n", aux_cnf_variable);
	}
	++Clause_cnt;


	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A_1.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s %s 0\n", "auxiliary", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d %d 0\n", aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", "auxiliary", spec_identifier_THEN_A_2.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1));
		}
	    }
	    ++Clause_cnt;
	}

	int N_Equalities = spec_Identifiers_THEN_A_3.size();
	sASSERT(spec_Identifiers_THEN_A_3.size() == spec_Identifiers_THEN_B_3.size());
	
	for (int equality = 0; equality < N_Equalities; ++equality)
	{
	    sASSERT(spec_Identifiers_THEN_A_3[equality].get_StateIdentifier()->get_Log2_StateCount() == spec_Identifiers_THEN_B_3[equality].get_StateIdentifier()->get_Log2_StateCount());
	    int N_THEN_3_Bits = spec_Identifiers_THEN_A_3[equality].get_StateIdentifier()->get_Log2_StateCount();
	    
	    for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s %s 0\n", "auxiliary", spec_Identifiers_THEN_A_3[equality].calc_String(bit_1).c_str(), spec_Identifiers_THEN_B_3[equality].calc_String(bit_1).c_str());
		    fprintf(fw, "-%s %s -%s 0\n", "auxiliary", spec_Identifiers_THEN_A_3[equality].calc_String(bit_1).c_str(), spec_Identifiers_THEN_B_3[equality].calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d %d 0\n", aux_cnf_variable, spec_Identifiers_THEN_A_3[equality].calc_CNF(bit_1), spec_Identifiers_THEN_B_3[equality].calc_CNF(bit_1));
		    fprintf(fw, "-%d %d -%d 0\n", aux_cnf_variable, spec_Identifiers_THEN_A_3[equality].calc_CNF(bit_1), spec_Identifiers_THEN_B_3[equality].calc_CNF(bit_1));
		}
		Clause_cnt += 2;
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
        #endif

	return Clause_cnt;
    }


    void sAdvancedClauseGenerator::cast_LargeConditionalEquality(Glucose::Solver                  *solver,
								 const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
								 int                               state_IF_B,
								 const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
								 int                               state_THEN_B_1,
								 const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
								 int                               state_THEN_B_2,
								 SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
								 SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3,
								 int                               sUNUSED(weight))
    {
	int Clause_cnt = 0;

	int N_IF_Bits = spec_identifier_IF_A.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_1_Bits = spec_identifier_THEN_A_1.get_StateIdentifier()->get_Log2_StateCount();
	int N_THEN_2_Bits = spec_identifier_THEN_A_2.get_StateIdentifier()->get_Log2_StateCount();

	int aux_cnf_variable = m_variable_store->get_Last_CNFVariable();
	m_variable_store->alloc_CNFVariables(1);

	std::vector<int> Literals;
	for (int bit_1 = 0; bit_1 < N_IF_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_IF_B) == 1)
	    {
		Literals.push_back(-spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	    else
	    {
		Literals.push_back(spec_identifier_IF_A.calc_CNF(bit_1));
	    }
	}
	Literals.push_back(aux_cnf_variable);
	cast_Clause(solver, Literals);
	++Clause_cnt;


	for (int bit_1 = 0; bit_1 < N_THEN_1_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_1) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_1.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

	for (int bit_1 = 0; bit_1 < N_THEN_2_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, state_THEN_B_2) == 1)
	    {
		cast_Clause(solver, -aux_cnf_variable, spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    else
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_identifier_THEN_A_2.calc_CNF(bit_1));
	    }
	    ++Clause_cnt;
	}

	int N_Equalities = spec_Identifiers_THEN_A_3.size();
	sASSERT(spec_Identifiers_THEN_A_3.size() == spec_Identifiers_THEN_B_3.size());
	
	for (int equality = 0; equality < N_Equalities; ++equality)
	{
	    sASSERT(spec_Identifiers_THEN_A_3[equality].get_StateIdentifier()->get_Log2_StateCount() == spec_Identifiers_THEN_B_3[equality].get_StateIdentifier()->get_Log2_StateCount());
	    int N_THEN_3_Bits = spec_Identifiers_THEN_A_3[equality].get_StateIdentifier()->get_Log2_StateCount();
	    
	    for (int bit_1 = 0; bit_1 < N_THEN_3_Bits; ++bit_1)
	    {
		cast_Clause(solver, -aux_cnf_variable, -spec_Identifiers_THEN_A_3[equality].calc_CNF(bit_1), spec_Identifiers_THEN_B_3[equality].calc_CNF(bit_1));
		cast_Clause(solver, -aux_cnf_variable, spec_Identifiers_THEN_A_3[equality].calc_CNF(bit_1), -spec_Identifiers_THEN_B_3[equality].calc_CNF(bit_1));
		Clause_cnt += 2;
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
        #endif
    }    


    int sAdvancedClauseGenerator::count_AllDifferenceConstraint(int                              &aux_Variable_cnt,
								int                              &total_Literal_cnt,
								SpecifiedStateIdentifiers_vector &spec_Identifiers) const
    {
	int Clause_cnt = 0;

	int N_Identifiers = spec_Identifiers.size();
	int N_Bits = spec_Identifiers[0].get_StateIdentifier()->get_Log2_StateCount();
	int N_States = spec_Identifiers[0].get_StateIdentifier()->get_StateCount();
	int log2_Identifiers = sIndexableStateIdentifier::calc_Log2(N_Identifiers);

	aux_Variable_cnt += 2 * N_Identifiers * (log2_Identifiers) + N_Identifiers * N_Bits + N_Identifiers * N_Identifiers * 2 + N_Bits * (N_Identifiers - 1);

	Clause_cnt += 2 * N_Identifiers * (sIndexableStateIdentifier::calc_Exp2(log2_Identifiers) - N_Identifiers) + N_Identifiers * (sIndexableStateIdentifier::calc_Exp2(N_Bits) - N_States);
	Clause_cnt += (N_Identifiers * N_Identifiers * ((1 + log2_Identifiers + 2 * N_Bits) + (1 + log2_Identifiers))) + (1 + (N_Bits * (N_Bits + 1))) * (N_Identifiers - 1);

	total_Literal_cnt += 0;
	sASSERT(false);

	return Clause_cnt;
    }


    int sAdvancedClauseGenerator::generate_AllDifferenceConstraint(FILE                             *fw,
								   SpecifiedStateIdentifiers_vector &spec_Identifiers,
								   bool                              string,
								   int                               weight)
    {
	int Clause_cnt = 0;
	int N_Identifiers = spec_Identifiers.size();
	int N_States = spec_Identifiers[0].get_StateIdentifier()->get_StateCount();

	sIndexableStateIdentifier bijection_A_auxiliary(m_variable_store, "bijection_A-" + sInt_32_to_String(m_aux_Identifier_cnt++), N_Identifiers, sIntegerScope(0, N_Identifiers - 1));
	sIndexableStateIdentifier bijection_B_auxiliary(m_variable_store, "bijection_B-" + sInt_32_to_String(m_aux_Identifier_cnt++), N_Identifiers, sIntegerScope(0, N_Identifiers - 1));
	sIndexableStateIdentifier ordered_auxiliary(m_variable_store, "ordered-" + sInt_32_to_String(m_aux_Identifier_cnt++), N_States, sIntegerScope(0, N_Identifiers - 1));

	for (int identifier = 0; identifier < N_Identifiers; ++identifier)
	{
	    generate_Alignment(fw, sSpecifiedStateIdentifier(&bijection_A_auxiliary, sIntegerIndex(identifier)), string, weight);
	    generate_Alignment(fw, sSpecifiedStateIdentifier(&bijection_B_auxiliary, sIntegerIndex(identifier)), string, weight);
	    generate_Alignment(fw, sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(identifier)), string, weight);
	}

	for (int identifier_A = 0; identifier_A < N_Identifiers; ++identifier_A)
	{
	    for (int identifier_B = 0; identifier_B < N_Identifiers; ++identifier_B)
	    {
		Clause_cnt += generate_ConditionalEquality(fw,
							   sSpecifiedStateIdentifier(&bijection_A_auxiliary, sIntegerIndex(identifier_A)),
							   identifier_B,
							   sSpecifiedStateIdentifier(&bijection_B_auxiliary, sIntegerIndex(identifier_B)),
							   identifier_A,
							   spec_Identifiers[identifier_A],
							   sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(identifier_B)),
							   string, 
							   weight);
		Clause_cnt += generate_ConditionalEquality(fw,
							   sSpecifiedStateIdentifier(&bijection_B_auxiliary, sIntegerIndex(identifier_B)),
							   identifier_A,
							   sSpecifiedStateIdentifier(&bijection_A_auxiliary, sIntegerIndex(identifier_A)),
							   identifier_B,							   
							   string,
							   weight);
	    }
	}

	for (int ordered = 0; ordered < N_Identifiers - 1; ++ordered)
	{
	    Clause_cnt += generate_LEXLess_Constraint(fw,
						      sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(ordered)),
						      sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(ordered + 1)),
						      string,
						      weight);
	}

	return Clause_cnt;
    }


    void sAdvancedClauseGenerator::cast_AllDifferenceConstraint(Glucose::Solver                  *solver,
								SpecifiedStateIdentifiers_vector &spec_Identifiers,
								int                               weight)
    {
	int N_Identifiers = spec_Identifiers.size();
	int N_States = spec_Identifiers[0].get_StateIdentifier()->get_StateCount();

	sIndexableStateIdentifier bijection_A_auxiliary(m_variable_store, "bijection_A-" + sInt_32_to_String(m_aux_Identifier_cnt++), N_Identifiers, sIntegerScope(0, N_Identifiers - 1));
	sIndexableStateIdentifier bijection_B_auxiliary(m_variable_store, "bijection_B-" + sInt_32_to_String(m_aux_Identifier_cnt++), N_Identifiers, sIntegerScope(0, N_Identifiers - 1));
	sIndexableStateIdentifier ordered_auxiliary(m_variable_store, "ordered-" + sInt_32_to_String(m_aux_Identifier_cnt++), N_States, sIntegerScope(0, N_Identifiers - 1));

	for (int identifier = 0; identifier < N_Identifiers; ++identifier)
	{
	    cast_Alignment(solver, sSpecifiedStateIdentifier(&bijection_A_auxiliary, sIntegerIndex(identifier)), weight);
	    cast_Alignment(solver, sSpecifiedStateIdentifier(&bijection_B_auxiliary, sIntegerIndex(identifier)), weight);
	    cast_Alignment(solver, sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(identifier)), weight);
	}

	for (int identifier_A = 0; identifier_A < N_Identifiers; ++identifier_A)
	{
	    for (int identifier_B = 0; identifier_B < N_Identifiers; ++identifier_B)
	    {
		cast_ConditionalEquality(solver,
					 sSpecifiedStateIdentifier(&bijection_A_auxiliary, sIntegerIndex(identifier_A)),
					 identifier_B,
					 sSpecifiedStateIdentifier(&bijection_B_auxiliary, sIntegerIndex(identifier_B)),
					 identifier_A,
					 spec_Identifiers[identifier_A],
					 sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(identifier_B)),
					 weight);
		cast_ConditionalEquality(solver,
					 sSpecifiedStateIdentifier(&bijection_B_auxiliary, sIntegerIndex(identifier_B)),
					 identifier_A,
					 sSpecifiedStateIdentifier(&bijection_A_auxiliary, sIntegerIndex(identifier_A)),
					 identifier_B,							   
					 weight);
	    }
	}

	for (int ordered = 0; ordered < N_Identifiers - 1; ++ordered)
	{
	    cast_LEXLess_Constraint(solver,
				    sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(ordered)),
				    sSpecifiedStateIdentifier(&ordered_auxiliary, sIntegerIndex(ordered + 1)),
				    weight);
	}
    }    


/*----------------------------------------------------------------------------*/
// sBitwiseClauseGenerator

    sBitwiseClauseGenerator::sBitwiseClauseGenerator(sVariableStore_CNF *variable_store)
	: sAdvancedClauseGenerator(variable_store)
    {
	// nothing
    }


    sBitwiseClauseGenerator::sBitwiseClauseGenerator(const sBitwiseClauseGenerator &clause_generator)
	: sAdvancedClauseGenerator(clause_generator)
    {
	// nothing
    }


    const sBitwiseClauseGenerator& sBitwiseClauseGenerator::operator=(const sBitwiseClauseGenerator &clause_generator)
    {
	sAdvancedClauseGenerator::operator=(clause_generator);
	return *this;
    }


    int sBitwiseClauseGenerator::count_Alignment(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	int N_States_1 = spec_identifier.get_StateIdentifier()->get_StateCount() - 1;

	for (int bit_1 = 0; bit_1 < N_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, N_States_1) == 0)
	    {
		++Clause_cnt;

		for (int bit_2 = N_Bits - 1; bit_2 > bit_1; --bit_2)
		{
		    ++total_Literal_cnt;
		}
		++total_Literal_cnt;
	    }
	}	

	return Clause_cnt;
    }


    int sBitwiseClauseGenerator::generate_Alignment(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string, int sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	int N_States_1 = spec_identifier.get_StateIdentifier()->get_StateCount() - 1;

	for (int bit_1 = 0; bit_1 < N_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, N_States_1) == 0)
	    {
		for (int bit_2 = N_Bits - 1; bit_2 > bit_1; --bit_2)
		{
		    if (calc_Bit(bit_2, N_States_1) == 0)
		    {
			if (string)
			{
			    fprintf(fw, "%s ", spec_identifier.calc_String(bit_2).c_str());
			}
			else
			{
			    fprintf(fw, "%d ", spec_identifier.calc_CNF(bit_2));
			}
		    }
		    else
		    {
			if (string)
			{
			    fprintf(fw, "-%s ", spec_identifier.calc_String(bit_2).c_str());
			}
			else
			{
			    fprintf(fw, "-%d ", spec_identifier.calc_CNF(bit_2));
			}
		    }
		}
		sASSERT(calc_Bit(bit_1, N_States_1) == 0);

		if (string)
		{
		    fprintf(fw, "-%s ", spec_identifier.calc_String(bit_1).c_str());
		}
		else
		{
		    fprintf(fw, "-%d ", spec_identifier.calc_CNF(bit_1));
		}
		++Clause_cnt;
		fprintf(fw, " 0\n");
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sBitwiseClauseGenerator::cast_Alignment(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	int N_States_1 = spec_identifier.get_StateIdentifier()->get_StateCount() - 1;

	for (int bit_1 = 0; bit_1 < N_Bits; ++bit_1)
	{
	    if (calc_Bit(bit_1, N_States_1) == 0)
	    {
		std::vector<int> Literals;
		for (int bit_2 = N_Bits - 1; bit_2 > bit_1; --bit_2)
		{
		    if (calc_Bit(bit_2, N_States_1) == 0)
		    {
			Literals.push_back(spec_identifier.calc_CNF(bit_2));
		    }
		    else
		    {
			Literals.push_back(-spec_identifier.calc_CNF(bit_2));
		    }
		}
		sASSERT(calc_Bit(bit_1, N_States_1) == 0);

		Literals.push_back(-spec_identifier.calc_CNF(bit_1));
		cast_Clause(solver, Literals);
		++Clause_cnt;
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif
    }    


    int sBitwiseClauseGenerator::count_Disequalities(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States) const
    {
	int Clause_cnt = 0;
	
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	sBinaryTree state_binary_tree;

	for (States_vector::const_iterator state = States.begin(); state != States.end(); ++state)
	{
	    sBinaryTree::Bits_list Bits;

	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		int state_bit = calc_Bit(bit, *state);
		sASSERT(state_bit == 1 || state_bit == 0);

		if (state_bit == 1)
		{
		    Bits.push_front(true);
		}
		else
		{
		    Bits.push_front(false);
		}
	    }
	    state_binary_tree.insert(Bits);
	}
	Clause_cnt += state_binary_tree.count_TreeDisequalities(aux_Variable_cnt, total_Literal_cnt, spec_identifier);
	return Clause_cnt;
    }


    int sBitwiseClauseGenerator::generate_Disequalities(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, bool string, int weight)
    {
	int Clause_cnt = 0;
	
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	sBinaryTree state_binary_tree;

	for (States_vector::const_iterator state = States.begin(); state != States.end(); ++state)
	{
	    sBinaryTree::Bits_list Bits;

	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		int state_bit = calc_Bit(bit, *state);
		sASSERT(state_bit == 1 || state_bit == 0);

		if (state_bit == 1)
		{
		    Bits.push_front(true);
		}
		else
		{
		    Bits.push_front(false);
		}
	    }
	    state_binary_tree.insert(Bits);
	}
	Clause_cnt += state_binary_tree.generate_TreeDisequalities(fw, spec_identifier, string, weight);
	return Clause_cnt;
    }


    void sBitwiseClauseGenerator::cast_Disequalities(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, int weight)
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	sBinaryTree state_binary_tree;

	for (States_vector::const_iterator state = States.begin(); state != States.end(); ++state)
	{
	    sBinaryTree::Bits_list Bits;

	    for (int bit = 0; bit < N_Bits; ++bit)
	    {
		int state_bit = calc_Bit(bit, *state);
		sASSERT(state_bit == 1 || state_bit == 0);

		if (state_bit == 1)
		{
		    Bits.push_front(true);
		}
		else
		{
		    Bits.push_front(false);
		}
	    }
	    state_binary_tree.insert(Bits);
	}
	state_binary_tree.cast_TreeDisequalities(solver, spec_identifier, weight);
    }    


/*----------------------------------------------------------------------------*/
// sBitClauseGenerator

    sBitClauseGenerator::sBitClauseGenerator(sVariableStore_CNF *variable_store)
	: m_variable_store(variable_store)
	, m_aux_Identifier_cnt(0)
    {
	m_auxiliary_Identifiers[sINT_32_MAX] = sIndexableIdentifier();
    }


    sBitClauseGenerator::sBitClauseGenerator(const sBitClauseGenerator &clause_generator)
	: m_variable_store(clause_generator.m_variable_store)
	, m_aux_Identifier_cnt(clause_generator.m_aux_Identifier_cnt)
	, m_auxiliary_Identifiers(clause_generator.m_auxiliary_Identifiers)
    {
	// nothing
    }


    const sBitClauseGenerator& sBitClauseGenerator::operator=(const sBitClauseGenerator &clause_generator)
    {
	m_variable_store = clause_generator.m_variable_store;
	m_aux_Identifier_cnt = clause_generator.m_aux_Identifier_cnt;
	m_auxiliary_Identifiers = clause_generator.m_auxiliary_Identifiers;

	return *this;
    }


/*----------------------------------------------------------------------------*/

    int sBitClauseGenerator::count_Equality(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, int sUNUSED(state)) const
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();
	total_Literal_cnt += N_Bits;

	return N_Bits;
    }


    int sBitClauseGenerator::generate_Equality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, int state, bool string, int sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    int state_bit = sStateClauseGenerator::calc_Bit(bit, state);
	    sASSERT(state_bit == 1 || state_bit == 0);

	    if (state_bit == 1)
	    {
		if (string)
		{
		    fprintf(fw, "%s 0\n", spec_identifier.calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "%d 0\n", spec_identifier.calc_CNF(bit));
		}
		++Clause_cnt;
	    }
	    else
	    {
		if (string)
		{
		    fprintf(fw, "-%s 0\n", spec_identifier.calc_String(bit).c_str());
		}
		else
		{
		    fprintf(fw, "-%d 0\n", spec_identifier.calc_CNF(bit));
		}
		++Clause_cnt;
	    }
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_Equality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int state, int sUNUSED(weight))
    {
	int N_Bits = spec_identifier.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_Bits; ++bit)
	{
	    int state_bit = sStateClauseGenerator::calc_Bit(bit, state);
	    sASSERT(state_bit == 1 || state_bit == 0);

	    if (state_bit == 1)
	    {
		cast_Clause(solver, spec_identifier.calc_CNF(bit));
	    }
	    else
	    {
		cast_Clause(solver, -spec_identifier.calc_CNF(bit));
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
  	    #endif	    
	}
    }    


    int sBitClauseGenerator::count_AllMutexConstraint(int                            &sUNUSED(aux_Variable_cnt),
						      int                            &total_Literal_cnt,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers) const
    {
	int N_Identifiers = spec_Identifiers.size();
	total_Literal_cnt += N_Identifiers * (N_Identifiers - 1);

	return N_Identifiers * (N_Identifiers - 1) / 2;
    }


    int sBitClauseGenerator::generate_AllMutexConstraint(FILE                           *fw,
							 SpecifiedBitIdentifiers_vector &spec_Identifiers,
							 bool                            string,
							 int                             sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Identifiers = spec_Identifiers.size();
	int N_Identifiers_1 = N_Identifiers - 1;

	typedef std::vector<int> CNFs_vector;
	CNFs_vector prec_IDs;
	prec_IDs.resize(N_Identifiers);

	for (int id = 0; id < N_Identifiers; ++id)
	{
	  prec_IDs[id] = spec_Identifiers[id].calc_CNF();
	}
	
	for (int id_A = 0; id_A < N_Identifiers_1; ++id_A)
	{
	    for (int id_B = id_A + 1; id_B < N_Identifiers; ++id_B)
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s 0\n", spec_Identifiers[id_A].calc_String().c_str(), spec_Identifiers[id_B].calc_String().c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d 0\n", prec_IDs[id_A], prec_IDs[id_B]);
		}
		++Clause_cnt;
	    }
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_AllMutexConstraint(Glucose::Solver                *solver,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers,
						      int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers.size();
	int N_Identifiers_1 = N_Identifiers - 1;

	typedef std::vector<int> CNFs_vector;
	CNFs_vector prec_IDs;
	prec_IDs.resize(N_Identifiers);

	for (int id = 0; id < N_Identifiers; ++id)
	{
	    prec_IDs[id] = spec_Identifiers[id].calc_CNF();
	}
	
	for (int id_A = 0; id_A < N_Identifiers_1; ++id_A)
	{
	    for (int id_B = id_A + 1; id_B < N_Identifiers; ++id_B)
	    {
		cast_Clause(solver, -prec_IDs[id_A], -prec_IDs[id_B]);

                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
		}
	        #endif		
	    }
	}
    }    


    int sBitClauseGenerator::count_LinearAllMutexConstraint(int                            &aux_Variable_cnt,
							    int                            &total_Literal_cnt,
							    SpecifiedBitIdentifiers_vector &spec_Identifiers) const
    {
	int N_Identifiers = spec_Identifiers.size();
	total_Literal_cnt += 9 * (N_Identifiers - 1) + 1;
	aux_Variable_cnt += 2 * N_Identifiers;

	return 4 * (N_Identifiers - 1) + 1;
    }


    int sBitClauseGenerator::generate_LinearAllMutexConstraint(FILE                           *fw,
							       SpecifiedBitIdentifiers_vector &spec_Identifiers,
							       bool                            string,
							       int                             sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Identifiers = spec_Identifiers.size();
	int N_Identifiers_1 = N_Identifiers - 1;

	sIndexableBitIdentifier partial_sum_auxiliary(m_variable_store, "partial_sum", sIntegerScope(0, 1), sIntegerScope(0, N_Identifiers_1));
	m_auxiliary_bit_Identifiers[partial_sum_auxiliary.get_First_CNFVariable()] = partial_sum_auxiliary;
    
	typedef std::vector<int> CNFs_vector;
	CNFs_vector prec_IDs;
	prec_IDs.resize(N_Identifiers);

	for (int id = 0; id < N_Identifiers; ++id)
	{
	    prec_IDs[id] = spec_Identifiers[id].calc_CNF();
	}
	
	for (int id = 0; id < N_Identifiers_1; ++id)
	{
	    if (string)
	    {
		fprintf(fw, "-%s %s 0\n",
			partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(id)).c_str(),
			partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(id + 1)).c_str());
		
		fprintf(fw, "-%s %s 0\n",
			partial_sum_auxiliary.calc_String(sIntegerIndex(1), sIntegerIndex(id)).c_str(),
			partial_sum_auxiliary.calc_String(sIntegerIndex(1), sIntegerIndex(id + 1)).c_str());
		
		fprintf(fw, "-%s %s 0\n", spec_Identifiers[id].calc_String().c_str(), partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(id)).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d %d 0\n",
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id)),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id + 1)));
		
		fprintf(fw, "-%d %d 0\n",
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(id)),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(id + 1)));
		
		fprintf(fw, "-%d %d 0\n", spec_Identifiers[id].calc_CNF(), partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id)));
	    }
	    Clause_cnt += 3;
	}
	for (int id = 1; id < N_Identifiers; ++id)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s %s 0\n",
			spec_Identifiers[id].calc_String().c_str(),
			partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(id - 1)).c_str(),
			partial_sum_auxiliary.calc_String(sIntegerIndex(1), sIntegerIndex(id)).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d %d 0\n",
			spec_Identifiers[id].calc_CNF(),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id - 1)),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(id)));
	    }
	    ++Clause_cnt;
	}
	if (string)
	{
//	    fprintf(fw, "%s 0\n", partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(N_Identifiers_1)).c_str());
	    fprintf(fw, "-%s 0\n", partial_sum_auxiliary.calc_String(sIntegerIndex(1), sIntegerIndex(N_Identifiers_1)).c_str());
	}
	else
	{
//	    fprintf(fw, "%d 0\n", partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(N_Identifiers_1)));
	    fprintf(fw, "-%d 0\n", partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(N_Identifiers_1)));
	}
	Clause_cnt += 2;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_LinearAllMutexConstraint(Glucose::Solver                *solver,
							    SpecifiedBitIdentifiers_vector &spec_Identifiers,
							    int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers.size();
	int N_Identifiers_1 = N_Identifiers - 1;

	sIndexableBitIdentifier partial_sum_auxiliary(m_variable_store, "partial_sum", sIntegerScope(0, 1), sIntegerScope(0, N_Identifiers_1));
	m_auxiliary_bit_Identifiers[partial_sum_auxiliary.get_First_CNFVariable()] = partial_sum_auxiliary;
    
	typedef std::vector<int> CNFs_vector;
	CNFs_vector prec_IDs;
	prec_IDs.resize(N_Identifiers);

	for (int id = 0; id < N_Identifiers; ++id)
	{
	    prec_IDs[id] = spec_Identifiers[id].calc_CNF();
	}
	
	for (int id = 0; id < N_Identifiers_1; ++id)
	{
	    cast_Clause(solver,
			-partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id)),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id + 1)));
		
	    cast_Clause(solver,
			-partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(id)),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(id + 1)));
		
	    cast_Clause(solver, -spec_Identifiers[id].calc_CNF(), partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id)));

            #ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 3;
	    }
 	    #endif	    	    
	}
	for (int id = 1; id < N_Identifiers; ++id)
	{
	    cast_Clause(solver,
			-spec_Identifiers[id].calc_CNF(),
			-partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(id - 1)),
			partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(id)));

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
 	    #endif	    
	}
	
	cast_Clause(solver, -partial_sum_auxiliary.calc_CNF(sIntegerIndex(1), sIntegerIndex(N_Identifiers_1)));

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    int sBitClauseGenerator::count_AdaptiveAllMutexConstraint(int                            &aux_Variable_cnt,
							      int                            &total_Literal_cnt,
							      SpecifiedBitIdentifiers_vector &spec_Identifiers) const
    {
	if (spec_Identifiers.size() <= 32)
//	if (spec_Identifiers.size() <= 512)
	{
	    return count_AllMutexConstraint(aux_Variable_cnt, total_Literal_cnt, spec_Identifiers);
	}
	else
	{	   
	    return count_LinearAllMutexConstraint(aux_Variable_cnt, total_Literal_cnt, spec_Identifiers);
	}
    }


    int sBitClauseGenerator::generate_AdaptiveAllMutexConstraint(FILE                           *fw,
								 SpecifiedBitIdentifiers_vector &spec_Identifiers,
								 bool                            string,
								 int                             weight)
    {
	if (spec_Identifiers.size() <= 32)
//	if (spec_Identifiers.size() <= 512)
	{
	    return generate_AllMutexConstraint(fw, spec_Identifiers, string, weight);
	}
	else
	{
	    return generate_LinearAllMutexConstraint(fw, spec_Identifiers, string, weight);
	}
    }


    void sBitClauseGenerator::cast_AdaptiveAllMutexConstraint(Glucose::Solver                *solver,
							      SpecifiedBitIdentifiers_vector &spec_Identifiers,
							      int                             weight)
    {
	if (spec_Identifiers.size() <= 32)
//	if (spec_Identifiers.size() <= 512)
	{
	    cast_AllMutexConstraint(solver, spec_Identifiers, weight);
	}
	else
	{
	    cast_LinearAllMutexConstraint(solver, spec_Identifiers, weight);
	}
    }    


    int sBitClauseGenerator::count_Disjunction(int                            &sUNUSED(aux_Variable_cnt),
					       int                            &total_Literal_cnt,
					       SpecifiedBitIdentifiers_vector &spec_Identifiers) const
    {
	int N_Identifiers = spec_Identifiers.size();
	total_Literal_cnt += N_Identifiers;

	return 1;
    }


    int sBitClauseGenerator::generate_Disjunction(FILE                           *fw,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers,
						  bool                            string,
						  int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers.size();
	
	for (int id = 0; id < N_Identifiers; ++id)
	{
	    if (string)
	    {
		fprintf(fw, "%s ", spec_Identifiers[id].calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "%d ", spec_Identifiers[id].calc_CNF());
	    }
	}
	fprintf(fw, "0\n");

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_Disjunction(Glucose::Solver                *solver,
					       SpecifiedBitIdentifiers_vector &spec_Identifiers,
					       int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers.size();
	std::vector<int> Literals;
	
	for (int id = 0; id < N_Identifiers; ++id)
	{
	    Literals.push_back(spec_Identifiers[id].calc_CNF());
	}
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    int sBitClauseGenerator::count_ConditionalAllMutexConstraint(int                            &sUNUSED(aux_Variable_cnt),
								 int                            &total_Literal_cnt,
								 const sSpecifiedBitIdentifier  &sUNUSED(spec_condition),
								 SpecifiedBitIdentifiers_vector &spec_Identifiers) const
    {
	int N_Identifiers = spec_Identifiers.size();
	total_Literal_cnt += 3 * N_Identifiers * (N_Identifiers - 1) / 2;

	return N_Identifiers * (N_Identifiers - 1) / 2;
    }


    int sBitClauseGenerator::generate_ConditionalAllMutexConstraint(FILE                           *fw,
								    const sSpecifiedBitIdentifier  &spec_condition,
								    SpecifiedBitIdentifiers_vector &spec_Identifiers,
								    bool                            string,
								    int                             sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_Identifiers = spec_Identifiers.size();
	int N_Identifiers_1 = N_Identifiers - 1;
	
	for (int id_A = 0; id_A < N_Identifiers_1; ++id_A)
	{
	    for (int id_B = id_A + 1; id_B < N_Identifiers; ++id_B)
	    {
		if (string)
		{
		    fprintf(fw, "-%s -%s -%s 0\n", spec_condition.calc_String().c_str(), spec_Identifiers[id_A].calc_String().c_str(), spec_Identifiers[id_B].calc_String().c_str());
		}
		else
		{
		    fprintf(fw, "-%d -%d -%d 0\n", spec_condition.calc_CNF(), spec_Identifiers[id_A].calc_CNF(), spec_Identifiers[id_B].calc_CNF());
		}
		++Clause_cnt;
	    }
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_ConditionalAllMutexConstraint(Glucose::Solver                *solver,
								 const sSpecifiedBitIdentifier  &spec_condition,
								 SpecifiedBitIdentifiers_vector &spec_Identifiers,
								 int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers.size();
	int N_Identifiers_1 = N_Identifiers - 1;
	
	for (int id_A = 0; id_A < N_Identifiers_1; ++id_A)
	{
	    for (int id_B = id_A + 1; id_B < N_Identifiers; ++id_B)
	    {
		{
		    cast_Clause(solver, -spec_condition.calc_CNF(), -spec_Identifiers[id_A].calc_CNF(), -spec_Identifiers[id_B].calc_CNF());
		    
                    #ifdef sSTATISTICS
		    {
			++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
		    }
	            #endif		    
		}
	    }
	}
    }    


    int sBitClauseGenerator::count_BitSet(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedBitIdentifier &sUNUSED(spec_identifier)) const
    {
	++total_Literal_cnt;

	return 1;
    }


    int sBitClauseGenerator::generate_BitSet(FILE *fw, const sSpecifiedBitIdentifier &spec_identifier, bool string, int sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "%s 0\n", spec_identifier.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "%d 0\n", spec_identifier.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_BitSet(Glucose::Solver *solver, const sSpecifiedBitIdentifier &spec_identifier, int sUNUSED(weight))
    {
	cast_Clause(solver, spec_identifier.calc_CNF());
	
        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    int sBitClauseGenerator::count_BitUnset(int &sUNUSED(aux_Variable_cnt), int &total_Literal_cnt, const sSpecifiedBitIdentifier &sUNUSED(spec_identifier)) const
    {
	++total_Literal_cnt;

	return 1;
    }


    int sBitClauseGenerator::generate_BitUnset(FILE *fw, const sSpecifiedBitIdentifier &spec_identifier, bool string, int sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s 0\n", spec_identifier.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d 0\n", spec_identifier.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_BitUnset(Glucose::Solver *solver, const sSpecifiedBitIdentifier &spec_identifier, int sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier.calc_CNF());

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    


    int sBitClauseGenerator::count_TriangleMutex(int                           &sUNUSED(aux_Variable_cnt),
						 int                           &total_Literal_cnt,
						 const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_A),
						 const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_B),
						 const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_C)) const
    {
	total_Literal_cnt += 3;

	return 1;
    }

	
    int sBitClauseGenerator::generate_TriangleMutex(FILE                          *fw,
						    const sSpecifiedBitIdentifier &spec_identifier_A,
						    const sSpecifiedBitIdentifier &spec_identifier_B,
						    const sSpecifiedBitIdentifier &spec_identifier_C,
						    bool                           string,
						    int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_A.calc_String().c_str(), spec_identifier_B.calc_String().c_str(), spec_identifier_C.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_A.calc_CNF(), spec_identifier_B.calc_CNF(), spec_identifier_C.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_TriangleMutex(Glucose::Solver               *solver,
						 const sSpecifiedBitIdentifier &spec_identifier_A,
						 const sSpecifiedBitIdentifier &spec_identifier_B,
						 const sSpecifiedBitIdentifier &spec_identifier_C,
						 int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_A.calc_CNF(), -spec_identifier_B.calc_CNF(), -spec_identifier_C.calc_CNF());

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    int sBitClauseGenerator::count_MultiTriangleMutex(int                            &sUNUSED(aux_Variable_cnt),
						      int                            &total_Literal_cnt,
						      const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_A),
						      const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_B),
						      SpecifiedBitIdentifiers_vector &spec_Identifiers_C) const
    {
	total_Literal_cnt += 3 * spec_Identifiers_C.size();

	return spec_Identifiers_C.size();
    }

	
    int sBitClauseGenerator::generate_MultiTriangleMutex(FILE                           *fw,
							 const sSpecifiedBitIdentifier  &spec_identifier_A,
							 const sSpecifiedBitIdentifier  &spec_identifier_B,
							 SpecifiedBitIdentifiers_vector &spec_Identifiers_C,
							 bool                            string,
							 int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_C = spec_Identifiers_C.begin(); spec_identifier_C != spec_Identifiers_C.end(); ++spec_identifier_C)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_A.calc_String().c_str(), spec_identifier_B.calc_String().c_str(), spec_identifier_C->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_A.calc_CNF(), spec_identifier_B.calc_CNF(), spec_identifier_C->calc_CNF());
	    }	    
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
#endif
	}
	
	return spec_Identifiers_C.size();
    }


    void sBitClauseGenerator::cast_MultiTriangleMutex(Glucose::Solver                *solver,
						      const sSpecifiedBitIdentifier  &spec_identifier_A,
						      const sSpecifiedBitIdentifier  &spec_identifier_B,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers_C,
						      int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_C = spec_Identifiers_C.begin(); spec_identifier_C != spec_Identifiers_C.end(); ++spec_identifier_C)
	{
	    cast_Clause(solver, -spec_identifier_A.calc_CNF(), -spec_identifier_B.calc_CNF(), -spec_identifier_C->calc_CNF());
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
#endif
	}
    }        


    int sBitClauseGenerator::count_BiangleMutex(int                           &sUNUSED(aux_Variable_cnt),
						int                           &total_Literal_cnt,
						const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_A),
						const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_B)) const
    {
	total_Literal_cnt += 2;

	return 1;
    }

	
    int sBitClauseGenerator::generate_BiangleMutex(FILE                          *fw,
						   const sSpecifiedBitIdentifier &spec_identifier_A,
						   const sSpecifiedBitIdentifier &spec_identifier_B,
						   bool                           string,
						   int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s -%s 0\n", spec_identifier_A.calc_String().c_str(), spec_identifier_B.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d -%d 0\n", spec_identifier_A.calc_CNF(), spec_identifier_B.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_BiangleMutex(Glucose::Solver               *solver,
						const sSpecifiedBitIdentifier &spec_identifier_A,
						const sSpecifiedBitIdentifier &spec_identifier_B,
						int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_A.calc_CNF(), -spec_identifier_B.calc_CNF());

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    int sBitClauseGenerator::count_MultiBiangleMutex(int                            &sUNUSED(aux_Variable_cnt),
						     int                            &total_Literal_cnt,
						     const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_A),
						     SpecifiedBitIdentifiers_vector &spec_Identifiers_B) const
    {
	total_Literal_cnt += spec_Identifiers_B.size() * 2;
	
	return spec_Identifiers_B.size();
    }

	
    int sBitClauseGenerator::generate_MultiBiangleMutex(FILE                           *fw,
							const sSpecifiedBitIdentifier  &spec_identifier_A,
							SpecifiedBitIdentifiers_vector &spec_Identifiers_B,
							bool                            string,
							int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_B = spec_Identifiers_B.begin(); spec_identifier_B != spec_Identifiers_B.end(); ++spec_identifier_B)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s 0\n", spec_identifier_A.calc_String().c_str(), spec_identifier_B->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d 0\n", spec_identifier_A.calc_CNF(), spec_identifier_B->calc_CNF());
	    }
		
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
#endif
	}

	return spec_Identifiers_B.size();
    }


    void sBitClauseGenerator::cast_MultiBiangleMutex(Glucose::Solver                *solver,
						     const sSpecifiedBitIdentifier  &spec_identifier_A,
						     SpecifiedBitIdentifiers_vector &spec_Identifiers_B,
						     int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_B = spec_Identifiers_B.begin(); spec_identifier_B != spec_Identifiers_B.end(); ++spec_identifier_B)
	{
	    cast_Clause(solver, -spec_identifier_A.calc_CNF(), -spec_identifier_B->calc_CNF());
		
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
#endif
	}
    }        

    

    int sBitClauseGenerator::count_Implication(int                           &sUNUSED(aux_Variable_cnt),
					       int                           &total_Literal_cnt,
					       const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC),
					       const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST)) const
    {
	total_Literal_cnt += 2;

	return 1;
    }



    int sBitClauseGenerator::generate_Implication(FILE                          *fw,
						  const sSpecifiedBitIdentifier &spec_identifier_PREC,
						  const sSpecifiedBitIdentifier &spec_identifier_POST,
						  bool                           string,
						  int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s %s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d %d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_Implication(Glucose::Solver               *solver,
					       const sSpecifiedBitIdentifier &spec_identifier_PREC,
					       const sSpecifiedBitIdentifier &spec_identifier_POST,
					       int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), spec_identifier_POST.calc_CNF());

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    int sBitClauseGenerator::count_Bimplication(int                           &sUNUSED(aux_Variable_cnt),
						int                           &total_Literal_cnt,
						const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_A),
						const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_B),						
						const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST)) const
    {
	total_Literal_cnt += 3;

	return 1;
    }



    int sBitClauseGenerator::generate_Bimplication(FILE                          *fw,
						   const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						   const sSpecifiedBitIdentifier &spec_identifier_PREC_B,						   
						   const sSpecifiedBitIdentifier &spec_identifier_POST,
						   bool                           string,
						   int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s -%s %s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d -%d %d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_Bimplication(Glucose::Solver               *solver,
						const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						const sSpecifiedBitIdentifier &spec_identifier_PREC_B,						
						const sSpecifiedBitIdentifier &spec_identifier_POST,
						int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST.calc_CNF());

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }    
    

    int sBitClauseGenerator::count_Implication(int                           &sUNUSED(aux_Variable_cnt),
					       int                           &total_Literal_cnt,
					       const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC),
					       const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST_A),
					       const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST_B)) const
    {
	total_Literal_cnt += 4;

	return 2;
    }



    int sBitClauseGenerator::generate_Implication(FILE                          *fw,
						  const sSpecifiedBitIdentifier &spec_identifier_PREC,
						  const sSpecifiedBitIdentifier &spec_identifier_POST_A,
						  const sSpecifiedBitIdentifier &spec_identifier_POST_B,
						  bool                           string,						  
						  int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s %s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST_A.calc_String().c_str());
	    fprintf(fw, "-%s %s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST_B.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d %d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST_A.calc_CNF());
	    fprintf(fw, "-%d %d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST_B.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif

	return 2;
    }


    void sBitClauseGenerator::cast_Implication(Glucose::Solver               *solver,
					       const sSpecifiedBitIdentifier &spec_identifier_PREC,
					       const sSpecifiedBitIdentifier &spec_identifier_POST_A,
					       const sSpecifiedBitIdentifier &spec_identifier_POST_B,
					       int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), spec_identifier_POST_A.calc_CNF());
	cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), spec_identifier_POST_B.calc_CNF());

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }    


    int sBitClauseGenerator::count_NonzeroImplication(int                             &sUNUSED(aux_Variable_cnt),
						      int                             &total_Literal_cnt,
						      const sSpecifiedStateIdentifier &spec_identifier_PREC,
						      const sSpecifiedBitIdentifier   &sUNUSED(spec_identifier_POST)) const
    {
	int N_PREC_Bits = spec_identifier_PREC.get_StateIdentifier()->get_Log2_StateCount();
	total_Literal_cnt += 2 * N_PREC_Bits;

	return N_PREC_Bits;
    }


    int sBitClauseGenerator::generate_NonzeroImplication(FILE                            *fw,
							 const sSpecifiedStateIdentifier &spec_identifier_PREC,
							 const sSpecifiedBitIdentifier   &spec_identifier_POST,
							 bool                             string,
							 int                              sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_PREC_Bits = spec_identifier_PREC.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_PREC_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "-%s %s 0\n", spec_identifier_PREC.calc_String(bit).c_str(), spec_identifier_POST.calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d %d 0\n", spec_identifier_PREC.calc_CNF(bit), spec_identifier_POST.calc_CNF());
	    }
	    ++Clause_cnt;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }

    
    void sBitClauseGenerator::cast_NonzeroImplication(Glucose::Solver                 *solver,
						      const sSpecifiedStateIdentifier &spec_identifier_PREC,
						      const sSpecifiedBitIdentifier   &spec_identifier_POST,
						      int                              sUNUSED(weight))
    {
	int N_PREC_Bits = spec_identifier_PREC.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_PREC_Bits; ++bit)
	{
	    cast_Clause(solver, -spec_identifier_PREC.calc_CNF(bit), spec_identifier_POST.calc_CNF());

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
 	    #endif	    
	}
    }    


    int sBitClauseGenerator::count_ZeroImplication(int                             &sUNUSED(aux_Variable_cnt),
						   int                             &total_Literal_cnt,
						   const sSpecifiedBitIdentifier   &sUNUSED(spec_identifier_PREC),
						   const sSpecifiedStateIdentifier &spec_identifier_POST) const
    {
	int N_POST_Bits = spec_identifier_POST.get_StateIdentifier()->get_Log2_StateCount();
	total_Literal_cnt += 2 * N_POST_Bits;

	return N_POST_Bits;
    }


    int sBitClauseGenerator::generate_ZeroImplication(FILE                            *fw,
						      const sSpecifiedBitIdentifier   &spec_identifier_PREC,
						      const sSpecifiedStateIdentifier &spec_identifier_POST,
						      bool                             string,
						      int                              sUNUSED(weight))
    {
	int Clause_cnt = 0;
	int N_POST_Bits = spec_identifier_POST.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_POST_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "%s -%s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST.calc_String(bit).c_str());
	    }
	    else
	    {
		fprintf(fw, "%d -%d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST.calc_CNF(bit));
	    }
	    ++Clause_cnt;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_ZeroImplication(Glucose::Solver                 *solver,
						   const sSpecifiedBitIdentifier   &spec_identifier_PREC,
						   const sSpecifiedStateIdentifier &spec_identifier_POST,
						   int                              sUNUSED(weight))
    {
	int N_POST_Bits = spec_identifier_POST.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_POST_Bits; ++bit)
	{
	    cast_Clause(solver, spec_identifier_PREC.calc_CNF(), -spec_identifier_POST.calc_CNF(bit));

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif	    
	}
    }


    int sBitClauseGenerator::count_Effect(int                           &sUNUSED(aux_Variable_cnt),
					  int                           &total_Literal_cnt,
					  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_A),
					  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_B),
					  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST)) const
    {
	total_Literal_cnt += 3;

	return 1;
    }


    int sBitClauseGenerator::generate_Effect(FILE                          *fw,
					     const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
					     const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
					     const sSpecifiedBitIdentifier &spec_identifier_POST,
					     bool                           string,
					     int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s -%s %s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d -%d %d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif

	return 2;
    }


    void sBitClauseGenerator::cast_Effect(Glucose::Solver               *solver,
					  const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
					  const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
					  const sSpecifiedBitIdentifier &spec_identifier_POST,
					  int                            sUNUSED(weight))
    {

	cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST.calc_CNF());

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }    


    int sBitClauseGenerator::count_MultiImplication(int                            &sUNUSED(aux_Variable_cnt),
						    int                            &total_Literal_cnt,
						    const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC),
						    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const
    {
	total_Literal_cnt += 1 + spec_Identifiers_POST.size();

	return 1;
    }
	

    int sBitClauseGenerator::generate_MultiImplication(FILE                           *fw,
						       const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						       SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						       bool                            string,
						       int                             sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s ", spec_identifier_PREC.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d ", spec_identifier_PREC.calc_CNF());
	}

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    if (string)
	    {
		fprintf(fw, "%s ", spec_identifier_POST->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "%d ", spec_identifier_POST->calc_CNF());
	    }
	}
	fprintf(fw, "0\n");

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif

	return 1;
    }


    void sBitClauseGenerator::cast_MultiImplication(Glucose::Solver                *solver,
						    const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						    int                             sUNUSED(weight))
    {
	std::vector<int> Literals;
	Literals.push_back(-spec_identifier_PREC.calc_CNF());

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    
	    Literals.push_back(spec_identifier_POST->calc_CNF());
	}
	cast_Clause(solver, Literals);

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	}
	#endif
    }


    int sBitClauseGenerator::count_MultiImpliedImplication(int                            &sUNUSED(aux_Variable_cnt),
							   int                            &total_Literal_cnt,
							   const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC),
							   SpecifiedBitIdentifiers_vector &sUNUSED(spec_Identifiers_MIDDLE),
							   SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const
    {
	total_Literal_cnt += 3 * spec_Identifiers_POST.size();

	return spec_Identifiers_POST.size();
    }
	

    int sBitClauseGenerator::generate_MultiImpliedImplication(FILE                           *fw,
							      const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							      SpecifiedBitIdentifiers_vector &spec_Identifiers_MIDDLE,							      
							      SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							      bool                            string,
							      int                             sUNUSED(weight))
    {
	SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_MIDDLE = spec_Identifiers_MIDDLE.begin(); 
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    sASSERT(spec_identifier_MIDDLE != spec_Identifiers_MIDDLE.end());
	    
	    if (string)
	    {
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_MIDDLE->calc_String().c_str(), spec_identifier_POST->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_MIDDLE->calc_CNF(), spec_identifier_POST->calc_CNF());
	    }
	    ++spec_identifier_MIDDLE;

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
   	    #endif	    
	}

	return spec_Identifiers_POST.size();
    }


    void sBitClauseGenerator::cast_MultiImpliedImplication(Glucose::Solver                *solver,
							   const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							   SpecifiedBitIdentifiers_vector &spec_Identifiers_MIDDLE,							   
							   SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							   int                             sUNUSED(weight))
    {
	SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_MIDDLE = spec_Identifiers_MIDDLE.begin(); 	
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    std::vector<int> Literals;
		
	    Literals.push_back(-spec_identifier_PREC.calc_CNF());
	    Literals.push_back(-spec_identifier_MIDDLE->calc_CNF());
	    Literals.push_back(spec_identifier_POST->calc_CNF());
	    cast_Clause(solver, Literals);

	    ++spec_identifier_MIDDLE;

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
  	    #endif	    
	}
    }    


    int sBitClauseGenerator::count_MultiConjunctiveImplication(int                            &sUNUSED(aux_Variable_cnt),
							       int                            &total_Literal_cnt,
							       const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC),
							       SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const
    {
	total_Literal_cnt += 2 * spec_Identifiers_POST.size();

	return spec_Identifiers_POST.size();
    }
	

    int sBitClauseGenerator::generate_MultiConjunctiveImplication(FILE                           *fw,
								  const sSpecifiedBitIdentifier  &spec_identifier_PREC,
								  SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
								  bool                            string,
								  int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    if (string)
	    {
		fprintf(fw, "-%s %s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d %d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST->calc_CNF());
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif	    
	}


	return spec_Identifiers_POST.size();	
    }


    void sBitClauseGenerator::cast_MultiConjunctiveImplication(Glucose::Solver                *solver,
							       const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							       SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							       int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), spec_identifier_POST->calc_CNF());
	    
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif
	}
    }        


    int sBitClauseGenerator::count_MultiNegation(int                            &sUNUSED(aux_Variable_cnt),
						 int                            &total_Literal_cnt,
						 SpecifiedBitIdentifiers_vector &spec_Identifiers) const
    {
	total_Literal_cnt += spec_Identifiers.size();

	return spec_Identifiers.size();
    }
	

    int sBitClauseGenerator::generate_MultiNegation(FILE                           *fw,
						    SpecifiedBitIdentifiers_vector &spec_Identifiers,
						    bool                            string, 
						    int                             sUNUSED(weight))
    {
	int Clause_cnt = 0;

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier = spec_Identifiers.begin(); spec_identifier != spec_Identifiers.end(); ++spec_identifier)
	{
	    if (string)
	    {
		fprintf(fw, "-%s 0\n", spec_identifier->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d 0\n", spec_identifier->calc_CNF());
	    }
	    ++Clause_cnt;

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif
	}
	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_MultiNegation(Glucose::Solver                *solver,
						 SpecifiedBitIdentifiers_vector &spec_Identifiers,
						 int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier = spec_Identifiers.begin(); spec_identifier != spec_Identifiers.end(); ++spec_identifier)
	{
	    {
		cast_Clause(solver, -spec_identifier->calc_CNF());
	    }

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
	    #endif
	}
    }    


    int sBitClauseGenerator::count_MultiNegativeImplication(int                            &sUNUSED(aux_Variable_cnt),
							    int                            &total_Literal_cnt,
							    const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC),
							    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const
    {
	total_Literal_cnt += 2 * spec_Identifiers_POST.size();

	return spec_Identifiers_POST.size();
    }
	

    int sBitClauseGenerator::generate_MultiNegativeImplication(FILE                           *fw,
							       const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							       SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							       bool                            string,
							       int                             sUNUSED(weight))
    {
	int Clause_cnt = 0;

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST->calc_CNF());
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
            #endif

	    ++Clause_cnt;
	}
	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_MultiNegativeImplication(Glucose::Solver                *solver,
							    const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							    int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), -spec_identifier_POST->calc_CNF());

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
            #endif
	}
    }    

    
    int sBitClauseGenerator::count_MultiExclusiveImplication(int                            &sUNUSED(aux_Variable_cnt),
							     int                            &total_Literal_cnt,
							     const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC),
							     SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const
    {
	total_Literal_cnt += 2 * spec_Identifiers_POST.size();

	return spec_Identifiers_POST.size();
    }


    int sBitClauseGenerator::generate_MultiExclusiveImplication(FILE                           *fw,
								const sSpecifiedBitIdentifier  &spec_identifier_PREC,
								SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
								bool                            string,
								int                             sUNUSED(weight))
    {
	int Clause_cnt = 0;

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    if (string)
	    {
		fprintf(fw, "%s -%s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_POST->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "%d -%d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_POST->calc_CNF());
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
            #endif

	    ++Clause_cnt;
	}
	return Clause_cnt;
    }


    void sBitClauseGenerator::cast_MultiExclusiveImplication(Glucose::Solver                *solver,
							     const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							     SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							     int                             sUNUSED(weight))
    {
	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    cast_Clause(solver, spec_identifier_PREC.calc_CNF(), -spec_identifier_POST->calc_CNF());

            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses;
	    }
            #endif
	}
    }    


    int sBitClauseGenerator::count_ConditionalEquality(int                             &sUNUSED(aux_Variable_cnt),
						       int                             &total_Literal_cnt,
						       const sSpecifiedBitIdentifier   &sUNUSED(spec_identifier_PREC),
						       const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						       const sSpecifiedStateIdentifier &sUNUSED(spec_identifier_THEN_B)) const
    {
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();
	total_Literal_cnt += 6 * N_THEN_Bits;

	return (2 * N_THEN_Bits);
    }

	
    int sBitClauseGenerator::generate_ConditionalEquality(FILE                            *fw,
							  const sSpecifiedBitIdentifier   &spec_identifier_PREC,
							  const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
							  const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
							  bool                             string,
							  int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount())

	int Clause_cnt = 0;
	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_THEN_Bits; ++bit)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s %s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str());
		fprintf(fw, "-%s %s -%s 0\n", spec_identifier_PREC.calc_String().c_str(), spec_identifier_THEN_A.calc_String(bit).c_str(), spec_identifier_THEN_B.calc_String(bit).c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d %d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit));
		fprintf(fw, "-%d %d -%d 0\n", spec_identifier_PREC.calc_CNF(), spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit));
	    }
	    Clause_cnt += 2;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += Clause_cnt;
	}
	#endif

	return Clause_cnt;
    }

    
    void sBitClauseGenerator::cast_ConditionalEquality(Glucose::Solver                 *solver,
						       const sSpecifiedBitIdentifier   &spec_identifier_PREC,
						       const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						       const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
						       int                              sUNUSED(weight))
    {
	sASSERT(spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount() == spec_identifier_THEN_B.get_StateIdentifier()->get_Log2_StateCount());

	int N_THEN_Bits = spec_identifier_THEN_A.get_StateIdentifier()->get_Log2_StateCount();

	for (int bit = 0; bit < N_THEN_Bits; ++bit)
	{
	    cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), -spec_identifier_THEN_A.calc_CNF(bit), spec_identifier_THEN_B.calc_CNF(bit));
	    cast_Clause(solver, -spec_identifier_PREC.calc_CNF(), spec_identifier_THEN_A.calc_CNF(bit), -spec_identifier_THEN_B.calc_CNF(bit));

            #ifdef sSTATISTICS
	    {
		s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	    }
	    #endif	    
	}
    }    


    int sBitClauseGenerator::count_SwapConstraint(int                           &sUNUSED(aux_Variable_cnt),
						  int                           &total_Literal_cnt,
						  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_A),
						  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_B),
						  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST_A),
						  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST_B)) const
    {
	total_Literal_cnt += 6;

	return 2;
    }


    int sBitClauseGenerator::generate_SwapConstraint(FILE                          *fw,
						     const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						     const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
						     const sSpecifiedBitIdentifier &spec_identifier_POST_A,
						     const sSpecifiedBitIdentifier &spec_identifier_POST_B,
						     bool                           string,
						     int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST_A.calc_String().c_str());
	    fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST_B.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST_A.calc_CNF());
	    fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST_B.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
	
	return 2;
    }


    void sBitClauseGenerator::cast_SwapConstraint(Glucose::Solver               *solver,
						  const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						  const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
						  const sSpecifiedBitIdentifier &spec_identifier_POST_A,
						  const sSpecifiedBitIdentifier &spec_identifier_POST_B,
						  int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), -spec_identifier_POST_A.calc_CNF());
	cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), -spec_identifier_POST_B.calc_CNF());

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }


    int sBitClauseGenerator::count_NegativeSwapConstraint(int                           &sUNUSED(aux_Variable_cnt),
							  int                           &total_Literal_cnt,
							  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_A),
							  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_PREC_B),
							  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST_A),
							  const sSpecifiedBitIdentifier &sUNUSED(spec_identifier_POST_B)) const
    {
	total_Literal_cnt += 6;

	return 2;
    }


    int sBitClauseGenerator::generate_NegativeSwapConstraint(FILE                          *fw,
							     const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
							     const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
							     const sSpecifiedBitIdentifier &spec_identifier_POST_A,
							     const sSpecifiedBitIdentifier &spec_identifier_POST_B,
							     bool                           string,
							     int                            sUNUSED(weight))
    {
	if (string)
	{
	    fprintf(fw, "-%s -%s %s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST_A.calc_String().c_str());
	    fprintf(fw, "-%s -%s %s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST_B.calc_String().c_str());
	}
	else
	{
	    fprintf(fw, "-%d -%d %d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST_A.calc_CNF());
	    fprintf(fw, "-%d -%d %d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST_B.calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif

	return 2;
    }

    
    void sBitClauseGenerator::cast_NegativeSwapConstraint(Glucose::Solver               *solver,
							  const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
							  const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
							  const sSpecifiedBitIdentifier &spec_identifier_POST_A,
							  const sSpecifiedBitIdentifier &spec_identifier_POST_B,
							  int                            sUNUSED(weight))
    {
	cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST_A.calc_CNF());
	cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST_B.calc_CNF());

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif
    }    


    int sBitClauseGenerator::count_SwapConstraint(int                            &sUNUSED(aux_Variable_cnt),
						  int                            &total_Literal_cnt,
						  const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC_A),
						  const sSpecifiedBitIdentifier  &sUNUSED(spec_identifier_PREC_B),
						  SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const
    {
	int N_Identifiers = spec_Identifiers_POST.size();
	total_Literal_cnt += 3 * N_Identifiers;

	return N_Identifiers;
    }


    int sBitClauseGenerator::generate_SwapConstraint(FILE                           *fw,
						     const sSpecifiedBitIdentifier  &spec_identifier_PREC_A,
						     const sSpecifiedBitIdentifier  &spec_identifier_PREC_B,
						     SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						     bool                            string,
						     int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers_POST.size();

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    if (string)
	    {
		fprintf(fw, "-%s -%s -%s 0\n", spec_identifier_PREC_A.calc_String().c_str(), spec_identifier_PREC_B.calc_String().c_str(), spec_identifier_POST->calc_String().c_str());
	    }
	    else
	    {
		fprintf(fw, "-%d -%d -%d 0\n", spec_identifier_PREC_A.calc_CNF(), spec_identifier_PREC_B.calc_CNF(), spec_identifier_POST->calc_CNF());
	    }
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += 2;
	}
	#endif

	return N_Identifiers;
    }


    void sBitClauseGenerator::cast_SwapConstraint(Glucose::Solver                *solver,
						  const sSpecifiedBitIdentifier  &spec_identifier_PREC_A,
						  const sSpecifiedBitIdentifier  &spec_identifier_PREC_B,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						  int                             sUNUSED(weight))
    {
	int N_Identifiers = spec_Identifiers_POST.size();

	for (SpecifiedBitIdentifiers_vector::const_iterator spec_identifier_POST = spec_Identifiers_POST.begin(); spec_identifier_POST != spec_Identifiers_POST.end(); ++spec_identifier_POST)
	{
	    cast_Clause(solver, -spec_identifier_PREC_A.calc_CNF(), -spec_identifier_PREC_B.calc_CNF(), -spec_identifier_POST->calc_CNF());
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.get_CurrentPhase().m_produced_cnf_Clauses += N_Identifiers;
	}
	#endif
    }    


    int sBitClauseGenerator::count_Cardinality(int                            &aux_Variable_cnt,
					       int                            &total_Literal_cnt,
					       SpecifiedBitIdentifiers_vector &spec_Identifiers,
					       int                             cardinality) const
    {
	aux_Variable_cnt += spec_Identifiers.size() * spec_Identifiers.size();

	int Clause_cnt = 0;

	total_Literal_cnt += 2;
	++Clause_cnt;
	
	for (int i = 1; i < spec_Identifiers.size(); ++i)
	{
	    for (int j = 0; j <= i; ++j)
	    {
		total_Literal_cnt += 2;
		++Clause_cnt;
	    }
	    total_Literal_cnt += 2;
	    ++Clause_cnt;
	    
	    for (int j = 1; j <= i; ++j)
	    {
		total_Literal_cnt += 3;
		++Clause_cnt;
	    }
	}
	for (int i = 0; i < spec_Identifiers.size(); ++i)
	{
	    for (int j = cardinality; j < spec_Identifiers.size(); ++j)
	    {
		total_Literal_cnt += 1;
		++Clause_cnt;
	    }
	}
	
	return Clause_cnt;
    }


    int sBitClauseGenerator::generate_Cardinality(FILE                           *fw,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers,
						  int                             cardinality,
						  bool                            string,
						  int                             sUNUSED(weight))
     {
	 sIndexableBitIdentifier partial_sum_auxiliary(m_variable_store, "partial_sum", sIntegerScope(0, spec_Identifiers.size() - 1), sIntegerScope(0, spec_Identifiers.size() - 1));
	 m_auxiliary_bit_Identifiers[partial_sum_auxiliary.get_First_CNFVariable()] = partial_sum_auxiliary;

	 int Clause_cnt = 0;

	 if (string)
	 {
	     fprintf(fw, "-%s %s 0\n",
		     spec_Identifiers[0].calc_String().c_str(), 
		     partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(0)).c_str());
	 }
	 else
	 {
	     fprintf(fw, "-%d %d 0\n",
		     spec_Identifiers[0].calc_CNF(), 
		     partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(0)));
#ifdef sDEBUG
/*
	     printf("-%s %s 0\n",
		    spec_Identifiers[0].calc_String().c_str(), 
		    partial_sum_auxiliary.calc_String(sIntegerIndex(0), sIntegerIndex(0)).c_str());
*/
#endif
	 }
	 ++Clause_cnt;

	 for (int i = 1; i < spec_Identifiers.size(); ++i)
	 {
	     for (int j = 0; j <= i; ++j)
	     {
		 if (string)
		 {
		     fprintf(fw, "-%s %s 0\n", partial_sum_auxiliary.calc_String(sIntegerIndex(i-1), sIntegerIndex(j)).c_str(), partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(j)).c_str());
		 }
		 else
		 {
		     fprintf(fw, "-%d %d 0\n", partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j)), partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
#ifdef sDEBUG
/*
		     printf("-%s %s 0\n", partial_sum_auxiliary.calc_String(sIntegerIndex(i-1), sIntegerIndex(j)).c_str(), partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(j)).c_str());
*/
#endif
		 }
		 ++Clause_cnt;
	     }
	     if (string)
	     {
		 fprintf(fw, "-%s %s 0\n",
			 spec_Identifiers[i].calc_String().c_str(), 
			 partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(0)).c_str());
	     }
	     else
	     {
		 fprintf(fw, "-%d %d 0\n",
			 spec_Identifiers[i].calc_CNF(), 
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(0)));

#ifdef sDEBUG
/*
		 printf("-%s %s 0\n",
			spec_Identifiers[i].calc_String().c_str(), 
			partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(0)).c_str());
*/
#endif
	     }
	     ++Clause_cnt;

	     for (int j = 1; j <= i; ++j)
	     {
		 if (string)
		 {
		     fprintf(fw, "-%s -%s %s 0\n",
			     spec_Identifiers[i].calc_String().c_str(), 
			     partial_sum_auxiliary.calc_String(sIntegerIndex(i-1), sIntegerIndex(j-1)).c_str(),
			     partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(j)).c_str());
		 }
		 else
		 {
		     fprintf(fw, "-%d -%d %d 0\n",
			     spec_Identifiers[i].calc_CNF(), 
			     partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j-1)),
			     partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
#ifdef sDEBUG
/*
		     printf("-%s -%s %s 0\n",
			    spec_Identifiers[i].calc_String().c_str(), 
			    partial_sum_auxiliary.calc_String(sIntegerIndex(i-1), sIntegerIndex(j-1)).c_str(),
			    partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(j)).c_str());
*/
#endif
		 }
		 ++Clause_cnt;
	     }
	 }
	 for (int i = 0; i < spec_Identifiers.size(); ++i)
	 {
	     for (int j = cardinality; j < spec_Identifiers.size(); ++j)
	     {
		 if (string)
		 {
		     fprintf(fw, "-%s 0\n",
			     partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(j)).c_str());
		 }
		 else
		 {
		     fprintf(fw, "-%d 0\n",
			     partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
#ifdef sDEBUG
/*
		     printf("-%s 0\n",
			    partial_sum_auxiliary.calc_String(sIntegerIndex(i), sIntegerIndex(j)).c_str());
*/
#endif
		 }
		 ++Clause_cnt;
	     }
	 }
	 return Clause_cnt;
     }


    void sBitClauseGenerator::cast_Cardinality(Glucose::Solver                *solver,
					       SpecifiedBitIdentifiers_vector &spec_Identifiers,
					       int                             cardinality,
					       int                             sUNUSED(weight))
    {
	 sIndexableBitIdentifier partial_sum_auxiliary(m_variable_store, "partial_sum", sIntegerScope(0, spec_Identifiers.size() - 1), sIntegerScope(0, spec_Identifiers.size() - 1));
	 m_auxiliary_bit_Identifiers[partial_sum_auxiliary.get_First_CNFVariable()] = partial_sum_auxiliary;

	 cast_Clause(solver, -spec_Identifiers[0].calc_CNF(), partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(0)));
	 /*
	 fprintf(fw, "-%d %d 0\n",
		 spec_Identifiers[0].calc_CNF(), 
		 partial_sum_auxiliary.calc_CNF(sIntegerIndex(0), sIntegerIndex(0)));
	 */

	 for (int i = 1; i < spec_Identifiers.size(); ++i)
	 {
	     for (int j = 0; j <= i; ++j)
	     {
		 cast_Clause(solver, -partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j)), partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
//		 fprintf(fw, "-%d %d 0\n", partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j)), partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
	     }
	     cast_Clause(solver, -spec_Identifiers[i].calc_CNF(), partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(0)));
/*	     
	     fprintf(fw, "-%d %d 0\n",
		     spec_Identifiers[i].calc_CNF(), 
		     partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(0)));
*/
	     for (int j = 1; j <= i; ++j)
	     {
		 cast_Clause(solver, -spec_Identifiers[i].calc_CNF(), -partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j-1)), partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
/*		 
		 fprintf(fw, "-%d -%d %d 0\n",
			 spec_Identifiers[i].calc_CNF(), 
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i-1), sIntegerIndex(j-1)),
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
*/
	     }
	 }
	 for (int i = 0; i < spec_Identifiers.size(); ++i)
	 {
	     for (int j = cardinality; j < spec_Identifiers.size(); ++j)
	     {
		 cast_Clause(solver, -partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
/*		 
		 fprintf(fw, "-%d 0\n",
			 partial_sum_auxiliary.calc_CNF(sIntegerIndex(i), sIntegerIndex(j)));
*/
	     }
	 }
    }   


/*----------------------------------------------------------------------------*/
    
    void sBitClauseGenerator::cast_Clause(Glucose::Solver *solver, int lit_1)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	
	cast_Clause(solver, Lits);
    }

    
    void sBitClauseGenerator::cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	
	cast_Clause(solver, Lits);
    }

    
    void sBitClauseGenerator::cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2, int lit_3)
    {
	vector<int> Lits;
	Lits.push_back(lit_1);
	Lits.push_back(lit_2);
	Lits.push_back(lit_3);
		
	cast_Clause(solver, Lits);
    }

    
    void sBitClauseGenerator::cast_Clause(Glucose::Solver *solver, vector<int> &Lits)
    {
	Glucose::vec<Glucose::Lit> glu_Lits;
	
	for (vector<int>::const_iterator lit = Lits.begin(); lit != Lits.end(); ++lit)
	{
	    int glu_var = sABS(*lit) - 1;
	    while (glu_var >= solver->nVars())
	    {
		solver->newVar();
	    }
	    glu_Lits.push((*lit > 0) ? Glucose::mkLit(glu_var) : ~Glucose::mkLit(glu_var));
	}
	solver->addClause(glu_Lits);
    }

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc
