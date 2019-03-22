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
/* cnf.h / 0.20-kruh_050                                                      */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF formula production tools.
//
/*----------------------------------------------------------------------------*/

#ifndef __CNF_H__
#define __CNF_H__

#include <vector>
#include <list>
#include <map>

#include <errno.h>
#include <signal.h>
#include <zlib.h>
#include <sys/resource.h>

#include "types.h"
#include "defs.h"
#include "result.h"

#include "glucose/System.h"
#include "glucose/ParseUtils.h"
#include "glucose/Options.h"
#include "glucose/Dimacs.h"

#include "glucose/Solver.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    class sIndexScope;

    class sIndexableIdentifier;
    class sSpecifiedIdentifier;

    class sIndexableStateIdentifier;
    class sSpecifiedStateIdentifier;

    class sIndexableBitIdentifier;
    class sSpecifiedBitIdentifier;




/*----------------------------------------------------------------------------*/
// sIndex

    class sIndex
    {
    public:
	virtual sIndex* clone(void) const = 0;

	virtual void increment(void) = 0;
	virtual void decrement(void) = 0;

	virtual void increment(int distance) = 0;
	virtual void decrement(int distance) = 0;

	virtual bool operator==(const sIndex &index) const = 0;
	virtual bool operator!=(const sIndex &index) const = 0;

	virtual bool operator<=(const sIndex &index) const = 0;
	virtual bool operator<(const sIndex &index) const = 0;
	virtual int operator-(const sIndex &index) const = 0;

	virtual bool operator>=(const sIndex &index) const;
	virtual bool operator>(const sIndex &index) const;

	virtual sString to_String(void) const = 0;
	virtual void to_Screen(void) const = 0;
    };


/*----------------------------------------------------------------------------*/
// sIntegerIndex

    class sIntegerIndex
    : public sIndex
    {
    public:
	sIntegerIndex();
	sIntegerIndex(int value);
	sIntegerIndex(const sIntegerIndex &integer_index);
	const sIntegerIndex& operator=(const sIntegerIndex &integer_index);

	virtual sIndex* clone(void) const;

	int get_Value(void) const;
	void set_Value(int value);

	virtual void increment(void);
	virtual void decrement(void);

	virtual void increment(int distance);
	virtual void decrement(int distance);

	virtual bool operator==(const sIndex &index) const;
	virtual bool operator!=(const sIndex &index) const;

	virtual bool operator<=(const sIndex &index) const;
	virtual bool operator<(const sIndex &index) const;
	virtual int operator-(const sIndex &index) const;

	virtual sString to_String(void) const;
	virtual void to_Screen(void) const;

    private:
	int m_value;
    };


/*----------------------------------------------------------------------------*/
// sIndex_iterator

    class sIndex_iterator
    {
    public:
	sIndex_iterator(const sIndex &index);
	sIndex_iterator(const sIndex_iterator &index_iterator);
	const sIndex_iterator& operator=(const sIndex_iterator &index_iterator);
	virtual ~sIndex_iterator();

	sIndex_iterator& operator++();
	sIndex_iterator& operator--();

	sIndex_iterator& operator+=(int distance);
	sIndex_iterator& operator-=(int distance);

	bool operator==(const sIndex_iterator &iterator) const;
	bool operator!=(const sIndex_iterator &iterator) const;

	const sIndex& operator*() const;

    private:
	sIndex *m_index;
    };


/*----------------------------------------------------------------------------*/
// sIndexScope

    class sIndexScope
    {
    public:
	sIndexScope();
	sIndexScope(const sIndexScope &scope);
	const sIndexScope& operator=(const sIndexScope &scope);

	virtual sIndexScope* clone(void) const = 0;
	virtual int get_Size(void) const = 0;

	virtual sIndex_iterator get_Begin(void) const = 0;
	virtual sIndex_iterator get_End(void) const = 0;
    };


/*----------------------------------------------------------------------------*/
// sIntegerScope

    class sIntegerScope
      : public sIndexScope
    {
    public:
	sIntegerScope();
	sIntegerScope(int min, int max);
	sIntegerScope(const sIntegerScope &integer_scope);
	const sIntegerScope& operator=(const sIntegerScope &integer_scope);

	virtual sIndexScope* clone(void) const;
	virtual int get_Size(void) const;
 
	virtual sIndex_iterator get_Begin(void) const;
	virtual sIndex_iterator get_End(void) const;

	inline int get_Min(void) const;
	inline int get_Max(void) const;

	inline void set_Min(int min);
	inline void set_Max(int max);

    private:
	int m_min;
	int m_max;
    };


/*----------------------------------------------------------------------------*/

    inline int sIntegerScope::get_Min(void) const
    {
	return m_min;
    }

    
    inline int sIntegerScope::get_Max(void) const
    {
	return m_max;
    }


    inline void sIntegerScope::set_Min(int min)
    {
	m_min = min;
    }


    inline void sIntegerScope::set_Max(int max)
    {
	m_max = max;
    }


/*----------------------------------------------------------------------------*/
// sVariableStore_CNF

    class sVariableStore_CNF
    {
    public:
	sVariableStore_CNF();
	sVariableStore_CNF(const sVariableStore_CNF &variable_store);
	const sVariableStore_CNF& operator=(const sVariableStore_CNF &variable_store);

	int get_Last_CNFVariable(void) const;
	void alloc_CNFVariables(int N_Variables);

    private:
	int m_last_variable;
    };


/*----------------------------------------------------------------------------*/
// sBinaryTreeNode

    class sBinaryTreeNode
    {
    public:
	friend class sBinaryTree;

    public:
	sBinaryTreeNode();
	virtual ~sBinaryTreeNode();

	virtual void to_Screen(const sString &indent = "") const;

    private:
	bool m_is_full;
	sBinaryTreeNode *m_prev_node;
	sBinaryTreeNode *m_left_node;
	sBinaryTreeNode *m_right_node;
    };


/*----------------------------------------------------------------------------*/
// sBinaryTree

    class sBinaryTree
    {
    public:
	typedef std::list<bool> Bits_list;

    public:
	sBinaryTree();
	~sBinaryTree();

	void insert(const Bits_list &Bits);

	int count_TreeDisequalities(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const;
	int generate_TreeDisequalities(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string = false, int weight = 0) const;
	void cast_TreeDisequalities(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int weight = 0);	

	int count_TreeDisequalities(const sBinaryTreeNode *tree_node, const Bits_list &prefix, int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const;
	int generate_TreeDisequalities(const sBinaryTreeNode *tree_node, const Bits_list &prefix, FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string = false, int weight = 0) const;
	void cast_TreeDisequalities(const sBinaryTreeNode *tree_node, const Bits_list &prefix, Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int weight = 0);

	/*----------------------------------------------------------------*/
	
	void cast_Clause(Glucose::Solver *solver, int lit_1);
	void cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2);
	void cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2, int lit_3);
	void cast_Clause(Glucose::Solver *solver, std::vector<int> &Lits);

	virtual void to_Screen(const sString &indent = "") const;

    private:
	sBinaryTreeNode *m_root_node;
    };


/*----------------------------------------------------------------------------*/
// sSpecifiedIdentifier

    class sSpecifiedIdentifier
    {
    public:
	typedef std::vector<const sIndex*> Indexes_vector;

    public:
	sSpecifiedIdentifier();
	sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier);
	sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const sIndex &index);
	sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const sIndex &index_1, const sIndex &index_2);
	sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const sIndex &index_1, const sIndex &index_2, const sIndex &index_3);
	sSpecifiedIdentifier(const sIndexableIdentifier *indexable_identifier, const Indexes_vector &scope_Indexes);

	sSpecifiedIdentifier(const sSpecifiedIdentifier &specified_identifier);
	const sSpecifiedIdentifier& operator=(const sSpecifiedIdentifier &specified_identifier);
	~sSpecifiedIdentifier();

	inline bool is_Null(void) const;
	inline const sIndexableIdentifier* get_IndexableIdentifier(void) const;
	inline Indexes_vector& get_ScopeIndexes(void);

	sString calc_String(void) const;
	int calc_CNF(void) const;

    private:
	const sIndexableIdentifier *m_indexable_identifier;
	Indexes_vector m_scope_Indexes;
    };


/*----------------------------------------------------------------------------*/

    inline bool sSpecifiedIdentifier::is_Null(void) const
    {
	return (m_indexable_identifier == NULL);
    }


    inline const sIndexableIdentifier* sSpecifiedIdentifier::get_IndexableIdentifier(void) const
    {
	return m_indexable_identifier;
    }


    inline sSpecifiedIdentifier::Indexes_vector& sSpecifiedIdentifier::get_ScopeIndexes(void)
    {
	return m_scope_Indexes;
    }


/*----------------------------------------------------------------------------*/
// sIndexableIdentifier

    class sIndexableIdentifier
    {
    public:
	typedef sSpecifiedIdentifier::Indexes_vector Indexes_vector;
	typedef std::vector<const sIndexScope*> IndexScopes_vector;

    public:
	sIndexableIdentifier();
	sIndexableIdentifier(sVariableStore_CNF *variable_store, const sString &base_name);
	sIndexableIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, const sIndexScope &scope);

	sIndexableIdentifier(sVariableStore_CNF *variable_store,
			   const sString      &base_name,
			   const sIndexScope  &scope_1,
			   const sIndexScope  &scope_2);

	sIndexableIdentifier(sVariableStore_CNF *variable_store,
			   const sString      &base_name,
			   const sIndexScope  &scope_1,
			   const sIndexScope  &scope_2,
			   const sIndexScope  &scope_3);

	sIndexableIdentifier(sVariableStore_CNF       *variable_store,
			   const sString             &base_name,
			   const IndexScopes_vector &index_Scopes);

	sIndexableIdentifier(const sIndexableIdentifier &indexable_identifier);
	const sIndexableIdentifier& operator=(const sIndexableIdentifier &indexable_identifier);
	~sIndexableIdentifier();

	inline bool is_Anonymous(void) const;
	inline const sString& get_BaseName(void) const;

	inline int get_Arity(void) const;
	inline int get_First_CNFVariable(void) const;
	inline int get_CNFVariable_Count(void) const;
	
	inline const sIndexScope& get_IndexScope(int scope) const;
	inline const IndexScopes_vector& get_IndexScopes(void) const;

	sString calc_String(void) const;
	sString calc_String(const sIndex &index) const;
	sString calc_String(const sIndex &index_1, const sIndex &index_2) const;
	sString calc_String(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const;
	sString calc_String(const Indexes_vector &scope_Indexes) const;

	int calc_CNF(void) const;
	int calc_CNF(const sIndex &index) const;
	int calc_CNF(const sIndex &index_1, const sIndex &index_2) const;
	int calc_CNF(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const;
	int calc_CNF(const Indexes_vector &scope_Indexes) const;

	sSpecifiedIdentifier translate_CNFVariable(int cnf_variable) const;

	virtual void to_Screen(const sString &indent = "") const;

    private:
	virtual void to_Screen_identifiers(const Indexes_vector &scope_Indexes, const sString &indent = "") const;

    private:
	sVariableStore_CNF *m_variable_store;
	sString m_base_name;

	int m_first_cnf_variable;
	int m_N_cnf_Variables;

	IndexScopes_vector m_index_Scopes;
    };


/*----------------------------------------------------------------------------*/

    inline bool sIndexableIdentifier::is_Anonymous(void) const
    {
	return (m_variable_store == NULL);
    }

    inline const sString& sIndexableIdentifier::get_BaseName(void) const
    {
	return m_base_name;
    }


    inline int sIndexableIdentifier::get_Arity(void) const
    {
	return m_index_Scopes.size();
    }


    inline int sIndexableIdentifier::get_First_CNFVariable(void) const
    {
	return m_first_cnf_variable;
    }


    inline int sIndexableIdentifier::get_CNFVariable_Count(void) const
    {
	return m_N_cnf_Variables;
    }


    inline const sIndexScope& sIndexableIdentifier::get_IndexScope(int scope) const
    {
	sASSERT(get_Arity() > scope);
	return *m_index_Scopes[scope];
    }


    inline  const sIndexableIdentifier::IndexScopes_vector& sIndexableIdentifier::get_IndexScopes(void) const
    {
	return m_index_Scopes;
    }


/*----------------------------------------------------------------------------*/
// sSpecifiedStateIdentifier

    class sSpecifiedStateIdentifier
    {
    public:
	typedef sIndexableIdentifier::Indexes_vector Indexes_vector;

    public:
	sSpecifiedStateIdentifier();
	sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier);
	sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const sIndex &index);
	sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const sIndex &index_1, const sIndex &index_2);
	sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const sIndex &index_1, const sIndex &index_2, const sIndex &index_3);
	sSpecifiedStateIdentifier(const sIndexableStateIdentifier *state_identifier, const Indexes_vector &scope_Indexes);

	sSpecifiedStateIdentifier(const sSpecifiedStateIdentifier &specified_identifier);
	const sSpecifiedStateIdentifier& operator=(const sSpecifiedStateIdentifier &specified_identifier);
	~sSpecifiedStateIdentifier();

	inline bool is_Null(void) const;
	inline const sIndexableStateIdentifier* get_StateIdentifier(void) const;

	sString calc_String(int bit) const;
	int calc_CNF(int bit) const;

    private:
	const sIndexableStateIdentifier *m_state_identifier;
	Indexes_vector m_scope_Indexes;
    };


/*----------------------------------------------------------------------------*/

    inline bool sSpecifiedStateIdentifier::is_Null(void) const
    {
	return (m_state_identifier == NULL);
    }


    inline const sIndexableStateIdentifier* sSpecifiedStateIdentifier::get_StateIdentifier(void) const
    {
	return m_state_identifier;
    }


/*----------------------------------------------------------------------------*/
// sIndexableStateIdentifier

    class sIndexableStateIdentifier
    {
	friend class sClauseGenerator;

    public:
	typedef sSpecifiedStateIdentifier::Indexes_vector Indexes_vector;
	typedef sIndexableIdentifier::IndexScopes_vector IndexScopes_vector;

    public:
	sIndexableStateIdentifier();
	sIndexableStateIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, int N_States);
	sIndexableStateIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, int N_States, const sIndexScope &scope);

	sIndexableStateIdentifier(sVariableStore_CNF *variable_store,
				const sString      &base_name,
				int                 N_States,
				const sIndexScope  &scope_1,
				const sIndexScope  &scope_2);

	sIndexableStateIdentifier(sVariableStore_CNF *variable_store,
				const sString      &base_name,
				int                 N_States,
				const sIndexScope  &scope_1,
				const sIndexScope  &scope_2,
				const sIndexScope  &scope_3);

	sIndexableStateIdentifier(sVariableStore_CNF       *variable_store,
				const sString            &base_name,
				int                       N_States,
				const IndexScopes_vector &index_Scopes);

	sIndexableStateIdentifier(const sIndexableStateIdentifier &state_identifier);
	const sIndexableStateIdentifier& operator=(const sIndexableStateIdentifier &state_identifier);
	~sIndexableStateIdentifier();

	inline bool is_Anonymous(void) const;
	inline int get_Arity(void) const;
	inline int get_First_CNFVariable(void) const;
	inline int get_CNFVariable_Count(void) const;

	inline int get_StateCount(void) const;
	inline int get_Log2_StateCount(void) const;

	inline const sIndexableIdentifier& get_IndexableIdentifier(void) const;

	sString calc_String(int bit) const;
	sString calc_String(const sIndex &index, int bit) const;
	sString calc_String(const sIndex &index_1, const sIndex &index_2, int bit) const;
	sString calc_String(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3, int bit) const;
	sString calc_String(const Indexes_vector &scope_Indexes, int bit) const;

	int calc_CNF(int bit) const;
	int calc_CNF(const sIndex &index, int bit) const;
	int calc_CNF(const sIndex &index_1, const sIndex &index_2, int bit) const;
	int calc_CNF(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3, int bit) const;
	int calc_CNF(const Indexes_vector &scope_Indexes, int bit) const;

	sSpecifiedIdentifier translate_CNFVariable(int cnf_variable) const;

	virtual void to_Screen(const sString &indent = "") const;

	static int calc_Log2(int value);
	static int calc_Exp2(int value);

    private:
	int m_N_States;
	int m_log_N_States;
	sIndexableIdentifier m_indexable_identifier;
    };


/*----------------------------------------------------------------------------*/

    inline bool sIndexableStateIdentifier::is_Anonymous(void) const
    {
	return m_indexable_identifier.is_Anonymous();
    }

    
    inline int sIndexableStateIdentifier::get_Arity(void) const
    {
	return m_indexable_identifier.get_Arity();
    }


    inline int sIndexableStateIdentifier::get_First_CNFVariable(void) const
    {
	return m_indexable_identifier.get_First_CNFVariable();
    }


    inline int sIndexableStateIdentifier::get_CNFVariable_Count(void) const
    {
	return m_indexable_identifier.get_CNFVariable_Count();
    }

    
    inline int sIndexableStateIdentifier::get_StateCount(void) const
    {
	return m_N_States;
    }


   inline int sIndexableStateIdentifier::get_Log2_StateCount(void) const
    {
	return m_log_N_States;
    }


    inline const sIndexableIdentifier& sIndexableStateIdentifier::get_IndexableIdentifier(void) const
    {
	return m_indexable_identifier;
    }


/*----------------------------------------------------------------------------*/
// sSpecifiedBitIdentifier

    class sSpecifiedBitIdentifier
    {
    public:
	typedef sIndexableIdentifier::Indexes_vector Indexes_vector;

    public:
	sSpecifiedBitIdentifier();
	sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier);
	sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const sIndex &index);
	sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const sIndex &index_1, const sIndex &index_2);
	sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const sIndex &index_1, const sIndex &index_2, const sIndex &index_3);
	sSpecifiedBitIdentifier(const sIndexableBitIdentifier *bit_identifier, const Indexes_vector &scope_Indexes);

	sSpecifiedBitIdentifier(const sSpecifiedBitIdentifier &specified_identifier);
	const sSpecifiedBitIdentifier& operator=(const sSpecifiedBitIdentifier &specified_identifier);
	~sSpecifiedBitIdentifier();

	inline bool is_Null(void) const;
	inline const sIndexableBitIdentifier* get_BitIdentifier(void) const;

	sString calc_String(void) const;
	int calc_CNF(void) const;

    private:
	const sIndexableBitIdentifier *m_bit_identifier;
	Indexes_vector m_scope_Indexes;
    };


/*----------------------------------------------------------------------------*/

    inline bool sSpecifiedBitIdentifier::is_Null(void) const
    {
	return (m_bit_identifier == NULL);
    }


    inline const sIndexableBitIdentifier* sSpecifiedBitIdentifier::get_BitIdentifier(void) const
    {
	return m_bit_identifier;
    }


/*----------------------------------------------------------------------------*/
// sIndexableBitIdentifier

    class sIndexableBitIdentifier
    {
	friend class sClauseGenerator;

    public:
	typedef sSpecifiedBitIdentifier::Indexes_vector Indexes_vector;
	typedef sIndexableIdentifier::IndexScopes_vector IndexScopes_vector;

    public:
	sIndexableBitIdentifier();
	sIndexableBitIdentifier(sVariableStore_CNF *variable_store, const sString &base_name);
	sIndexableBitIdentifier(sVariableStore_CNF *variable_store, const sString &base_name, const sIndexScope &scope);

	sIndexableBitIdentifier(sVariableStore_CNF *variable_store,
				const sString      &base_name,
				const sIndexScope  &scope_1,
				const sIndexScope  &scope_2);

	sIndexableBitIdentifier(sVariableStore_CNF *variable_store,
				const sString      &base_name,
				const sIndexScope  &scope_1,
				const sIndexScope  &scope_2,
				const sIndexScope  &scope_3);

	sIndexableBitIdentifier(sVariableStore_CNF       *variable_store,
				const sString            &base_name,
				const IndexScopes_vector &index_Scopes);

	sIndexableBitIdentifier(const sIndexableBitIdentifier &bit_identifier);
	const sIndexableBitIdentifier& operator=(const sIndexableBitIdentifier &bit_identifier);
	~sIndexableBitIdentifier();

	inline bool is_Anonymous(void) const;
	inline int get_Arity(void) const;
	inline int get_First_CNFVariable(void) const;
	inline int get_CNFVariable_Count(void) const;

	inline const sIndexableIdentifier& get_IndexableIdentifier(void) const;

	sString calc_String(void) const;
	sString calc_String(const sIndex &index) const;
	sString calc_String(const sIndex &index_1, const sIndex &index_2) const;
	sString calc_String(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const;
	sString calc_String(const Indexes_vector &scope_Indexes) const;

	int calc_CNF(void) const;
	int calc_CNF(const sIndex &index) const;
	int calc_CNF(const sIndex &index_1, const sIndex &index_2) const;
	int calc_CNF(const sIndex &index_1, const sIndex &index_2, const sIndex &index_3) const;
	int calc_CNF(const Indexes_vector &scope_Indexes) const;

	sSpecifiedIdentifier translate_CNFVariable(int cnf_variable) const;

	virtual void to_Screen(const sString &indent = "") const;

    private:
	sIndexableIdentifier m_indexable_identifier;
    };


/*----------------------------------------------------------------------------*/

    inline bool sIndexableBitIdentifier::is_Anonymous(void) const
    {
	return m_indexable_identifier.is_Anonymous();
    }

    
    inline int sIndexableBitIdentifier::get_Arity(void) const
    {
	return m_indexable_identifier.get_Arity();
    }


    inline int sIndexableBitIdentifier::get_First_CNFVariable(void) const
    {
	return m_indexable_identifier.get_First_CNFVariable();
    }


    inline int sIndexableBitIdentifier::get_CNFVariable_Count(void) const
    {
	return m_indexable_identifier.get_CNFVariable_Count();
    }

    
    inline const sIndexableIdentifier& sIndexableBitIdentifier::get_IndexableIdentifier(void) const
    {
	return m_indexable_identifier;
    }


/*----------------------------------------------------------------------------*/
// sStateClauseGenerator

    class sStateClauseGenerator
    {
    public:
	typedef std::vector<sSpecifiedStateIdentifier> SpecifiedStateIdentifiers_vector;
	typedef std::vector<int> States_vector;
	typedef std::map<int, sIndexableIdentifier, std::less<int> > AuxiliaryIdentifiers_map;

    public:
	sStateClauseGenerator(sVariableStore_CNF *variable_store);
	sStateClauseGenerator(const sStateClauseGenerator &clause_generator);
	const sStateClauseGenerator& operator=(const sStateClauseGenerator &clause_generator);

	virtual int count_Alignment(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const;
	virtual int generate_Alignment(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string = false, int weight = 0);
	virtual void cast_Alignment(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int weight = 0);	

	virtual int count_Equality(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, int state) const;
	virtual int generate_Equality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, int state, bool string = false, int weight = 0);
	virtual void cast_Equality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int state, int weight = 0);	

	virtual int count_Disequality(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, int state) const;
	virtual int generate_Disequality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, int state, bool string = false, int weight = 0);
	virtual void cast_Disequality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int state, int weight = 0);	

	virtual int count_Disequalities(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States) const;
	virtual int generate_Disequalities(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, bool string = false, int weight = 0);
	virtual void cast_Disequalities(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, int weight = 0);	

	virtual int count_DisjunctiveDisequality(int                             &aux_Variable_cnt,
						 int                             &total_Literal_cnt,
						 const sSpecifiedStateIdentifier &spec_identifier_A,
						 int                              state_A,
						 const sSpecifiedStateIdentifier &spec_identifier_B,
						 int                              state_B) const;
	virtual int generate_DisjunctiveDisequality(FILE                            *fw,
						    const sSpecifiedStateIdentifier &spec_identifier_A,
						    int                              state_A,
						    const sSpecifiedStateIdentifier &spec_identifier_B,
						    int                              state_B,
						    bool                             string = false,
						    int                              weight = 0);
	virtual void cast_DisjunctiveDisequality(Glucose::Solver                 *solver,
						 const sSpecifiedStateIdentifier &spec_identifier_A,
						 int                              state_A,
						 const sSpecifiedStateIdentifier &spec_identifier_B,
						 int                              state_B,
						 int                              weight = 0);	

	virtual int count_Equality(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier_A, const sSpecifiedStateIdentifier &spec_identifier_B) const;
	virtual int generate_Equality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier_A, const sSpecifiedStateIdentifier &spec_identifier_B, bool string = false, int weight = 0);
	virtual void cast_Equality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier_A, const sSpecifiedStateIdentifier &spec_identifier_B, int weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
					      int                              weight = 0);
     

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B) const;       
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      int                              state_THEN_B) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						 int                              state_THEN_B,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      int                              state_THEN_B,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2) const;
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_1,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      int                              state_THEN_B_2) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
						 int                              state_THEN_B_1,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
						 int                              state_THEN_B_2,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      int                              state_THEN_B_2,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2) const;       
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
						 int                              state_THEN_B_1,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
					      int                              weight = 0);	
	
	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      int                              state_THEN_B_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_3) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
						 int                              state_THEN_B_1,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
						 int                              state_THEN_B_2,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_3,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      int                              state_THEN_B_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_3,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      int                              state_THEN_B_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
					      int                              state_THEN_B_3,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_4) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
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
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
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
					      int                              weight = 0);	

	virtual int count_DifferenceConstraint(int                              &aux_Variable_cnt,
					       int                              &total_Literal_cnt,
					       const sSpecifiedStateIdentifier  &spec_identifier_A,
					       SpecifiedStateIdentifiers_vector &spec_Identifiers_B) const;
	virtual int generate_DifferenceConstraint(FILE                             *fw,
						  const sSpecifiedStateIdentifier  &spec_identifier_A,
						  SpecifiedStateIdentifiers_vector &spec_Identifiers_B,
						  bool                              string = false,
						  int                               weight = 0);
	virtual void cast_DifferenceConstraint(Glucose::Solver                  *solver,
					       const sSpecifiedStateIdentifier  &spec_identifier_A,
					       SpecifiedStateIdentifiers_vector &spec_Identifiers_B,
					       int                               weight = 0);	

	virtual int count_AllDifferenceConstraint(int                              &aux_Variable_cnt,
						  int                              &total_Literal_cnt,
						  SpecifiedStateIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_AllDifferenceConstraint(FILE                             *fw,
						     SpecifiedStateIdentifiers_vector &spec_Identifiers,
						     bool                              string = false,
						     int                               weight = 0);
	virtual void cast_AllDifferenceConstraint(Glucose::Solver                  *solver,
						  SpecifiedStateIdentifiers_vector &spec_Identifiers,
						  int                               weight = 0);	

	virtual int count_CaseSplitting(int                              &aux_Variable_cnt,
					int                              &total_Literal_cnt,
					SpecifiedStateIdentifiers_vector &split_Identifiers,
					States_vector                    &split_States) const;
	virtual int generate_CaseSplitting(FILE                             *fw,
					   SpecifiedStateIdentifiers_vector &split_Identifiers,
					   States_vector                    &split_States,
					   bool                              string = false,
					   int                               weight = 0);
	virtual void cast_CaseSplitting(Glucose::Solver                  *solver,
					SpecifiedStateIdentifiers_vector &split_Identifiers,
					States_vector                    &split_States,
					int                               weight = 0);	

	virtual int count_DisjunctiveEquality(int                              &aux_Variable_cnt,
					      int                              &total_Literal_cnt,
					      SpecifiedStateIdentifiers_vector &disj_Identifiers,
					      States_vector                    &disj_States) const;
	virtual int generate_DisjunctiveEquality(FILE                             *fw,
						 SpecifiedStateIdentifiers_vector &disj_Identifiers,
						 States_vector                    &disj_States,
						 bool                              string = false,
						 int                               weight = 0);
	virtual void cast_DisjunctiveEquality(Glucose::Solver                  *solver,
					      SpecifiedStateIdentifiers_vector &disj_Identifiers,
					      States_vector                    &disj_States,
					      int                               weight = 0);	

	virtual int count_LargeConditionalEquality(int                              &aux_Variable_cnt,
						   int                              &total_Literal_cnt,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B);
	virtual int generate_LargeConditionalEquality(FILE                              *fw,
						      const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						      int                               state_IF_B,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B,
						      bool                              string = false,
						      int                               weight = 0);
	virtual void cast_LargeConditionalEquality(Glucose::Solver                  *solver,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B,
						   int                               weight = 0);	

	virtual int count_LargeConditionalEquality(int                              &aux_Variable_cnt,
						   int                              &total_Literal_cnt,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
						   int                               state_THEN_B_1,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
						   int                               state_THEN_B_2,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3);
	virtual int generate_LargeConditionalEquality(FILE                              *fw,
						      const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						      int                               state_IF_B,
						      const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
						      int                               state_THEN_B_1,
						      const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
						      int                               state_THEN_B_2,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3,
						      bool                              string = false,
						      int                               weight = 0);
	virtual void cast_LargeConditionalEquality(Glucose::Solver                  *solver,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
						   int                               state_THEN_B_1,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
						   int                               state_THEN_B_2,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3,
						   int                               weight = 0);	

	virtual int count_LEXLess_Constraint(int                             &aux_Variable_cnt,
					     int                             &total_Literal_cnt,
					     const sSpecifiedStateIdentifier &spec_identifier_A,
					     const sSpecifiedStateIdentifier &spec_identifier_B) const;	
	virtual int generate_LEXLess_Constraint(FILE                            *fw,
						const sSpecifiedStateIdentifier &spec_identifier_A,
						const sSpecifiedStateIdentifier &spec_identifier_B,
						bool                             string = false,
						int                              weight = 0);
	virtual void cast_LEXLess_Constraint(Glucose::Solver                 *solver,
					     const sSpecifiedStateIdentifier &spec_identifier_A,
					     const sSpecifiedStateIdentifier &spec_identifier_B,
					     int                              weight = 0);	

	sSpecifiedIdentifier translate_AuxiliaryCNFVariable(int cnf_variable) const;

	static int calc_Bit(int bit, int state);

	/*----------------------------------------------------------------*/
	
	void cast_Clause(Glucose::Solver *solver, int lit_1);
	void cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2);
	void cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2, int lit_3);
	void cast_Clause(Glucose::Solver *solver, std::vector<int> &Lits);	

    protected:
	sVariableStore_CNF *m_variable_store;

	int m_aux_Identifier_cnt;
	AuxiliaryIdentifiers_map m_auxiliary_Identifiers;
    };


/*----------------------------------------------------------------------------*/
// sAdvancedClauseGenerator

    class sAdvancedClauseGenerator
    : public sStateClauseGenerator
    {
    public:
	sAdvancedClauseGenerator(sVariableStore_CNF *variable_store);
	sAdvancedClauseGenerator(const sAdvancedClauseGenerator &clause_generator);
	const sAdvancedClauseGenerator& operator=(const sAdvancedClauseGenerator &clause_generator);

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      int                              state_THEN_B) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
						 int                              state_THEN_B_1,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2) const;	
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedStateIdentifier &spec_identifier_IF_A,
						 int                              state_IF_B,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
						 int                              state_THEN_B_1,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_2,
					      int                              weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedStateIdentifier &spec_identifier_IF_A,
					      int                              state_IF_B,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_1,
					      int                              state_THEN_B_1,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_2,
					      int                              state_THEN_B_2,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_3,
					      int                              state_THEN_B_3,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A_4,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B_4) const;       
	virtual int generate_ConditionalEquality(FILE                            *fw,
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
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
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
					      int                              weight = 0);	
	
	virtual int count_LargeConditionalEquality(int                              &aux_Variable_cnt,
						   int                              &total_Literal_cnt,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B);
	virtual int generate_LargeConditionalEquality(FILE                              *fw,
						      const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						      int                               state_IF_B,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B,
						      bool                              string = false,
						      int                               weight = 0);
	virtual void cast_LargeConditionalEquality(Glucose::Solver                  *solver,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B,
						   int                               weight = 0);

	
	virtual int count_LargeConditionalEquality(int                              &aux_Variable_cnt,
						   int                              &total_Literal_cnt,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
						   int                               state_THEN_B_1,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
						   int                               state_THEN_B_2,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3);
	virtual int generate_LargeConditionalEquality(FILE                              *fw,
						      const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						      int                               state_IF_B,
						      const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
						      int                               state_THEN_B_1,
						      const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
						      int                               state_THEN_B_2,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
						      SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3,
						      bool                              string = false,
						      int                               weight = 0);
	virtual void cast_LargeConditionalEquality(Glucose::Solver                  *solver,
						   const sSpecifiedStateIdentifier  &spec_identifier_IF_A,
						   int                               state_IF_B,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_1,
						   int                               state_THEN_B_1,
						   const sSpecifiedStateIdentifier  &spec_identifier_THEN_A_2,
						   int                               state_THEN_B_2,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_A_3,
						   SpecifiedStateIdentifiers_vector &spec_Identifiers_THEN_B_3,
						   int                               weight = 0);	

	virtual int count_AllDifferenceConstraint(int                              &aux_Variable_cnt,
						  int                              &total_Literal_cnt,
						  SpecifiedStateIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_AllDifferenceConstraint(FILE                             *fw,
						     SpecifiedStateIdentifiers_vector &spec_Identifiers,
						     bool                              string = false,
						     int                               weight = 0);
	virtual void cast_AllDifferenceConstraint(Glucose::Solver                  *solver,
						  SpecifiedStateIdentifiers_vector &spec_Identifiers,
						  int                               weight = 0);	
    };


/*----------------------------------------------------------------------------*/
// sBitwiseClauseGenerator

    class sBitwiseClauseGenerator
    : public sAdvancedClauseGenerator
    {
    public:
	sBitwiseClauseGenerator(sVariableStore_CNF *variable_store);
	sBitwiseClauseGenerator(const sBitwiseClauseGenerator &clause_generator);
	const sBitwiseClauseGenerator& operator=(const sBitwiseClauseGenerator &clause_generator);

	virtual int count_Alignment(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier) const;
	virtual int generate_Alignment(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, bool string = false, int weight = 0);
	virtual void cast_Alignment(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int weight = 0);	

	virtual int count_Disequalities(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States) const;
	virtual int generate_Disequalities(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, bool string = false, int weight = 0);
	virtual void cast_Disequalities(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, const States_vector &States, int weight = 0);	
    };


/*----------------------------------------------------------------------------*/
// sBitClauseGenerator

    class sBitClauseGenerator
    {
    public:
	typedef std::vector<sSpecifiedBitIdentifier> SpecifiedBitIdentifiers_vector;
	typedef std::vector<int> Bits_vector;
	typedef std::map<int, sIndexableIdentifier, std::less<int> > AuxiliaryIdentifiers_map;
	typedef std::map<int, sIndexableBitIdentifier, std::less<int> > AuxiliaryBitIdentifiers_map;

    public:
	sBitClauseGenerator(sVariableStore_CNF *variable_store);
	sBitClauseGenerator(const sBitClauseGenerator &clause_generator);
	const sBitClauseGenerator& operator=(const sBitClauseGenerator &clause_generator);

	virtual int count_Equality(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedStateIdentifier &spec_identifier, int state) const;
	virtual int generate_Equality(FILE *fw, const sSpecifiedStateIdentifier &spec_identifier, int state, bool string = false, int weight = 0);
	virtual void cast_Equality(Glucose::Solver *solver, const sSpecifiedStateIdentifier &spec_identifier, int state, int weight = 0);

	virtual int count_AllMutexConstraint(int                            &aux_Variable_cnt,
					     int                            &total_Literal_cnt,
					     SpecifiedBitIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_AllMutexConstraint(FILE                           *fw,
						SpecifiedBitIdentifiers_vector &spec_Identifiers,
						bool                            string = false,
						int                             weight = 0);
	virtual void cast_AllMutexConstraint(Glucose::Solver                *solver,
					    SpecifiedBitIdentifiers_vector &spec_Identifiers,
					    int                             weight = 0);	

	virtual int count_LinearAllMutexConstraint(int                            &aux_Variable_cnt,
						   int                            &total_Literal_cnt,
						   SpecifiedBitIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_LinearAllMutexConstraint(FILE                           *fw,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers,
						      bool                            string = false,
						      int                             weight = 0);
	virtual void cast_LinearAllMutexConstraint(Glucose::Solver                *solver,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers,
						  int                             weight = 0);	

	virtual int count_AdaptiveAllMutexConstraint(int                            &aux_Variable_cnt,
						     int                            &total_Literal_cnt,
						     SpecifiedBitIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_AdaptiveAllMutexConstraint(FILE                           *fw,
							SpecifiedBitIdentifiers_vector &spec_Identifiers,
							bool                            string = false,
							int                             weight = 0);
	virtual void cast_AdaptiveAllMutexConstraint(Glucose::Solver                *solver,
						    SpecifiedBitIdentifiers_vector &spec_Identifiers,
						    int                             weight = 0);	

	virtual int count_Disjunction(int                            &aux_Variable_cnt,
				      int                            &total_Literal_cnt,
				      SpecifiedBitIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_Disjunction(FILE                           *fw,
					 SpecifiedBitIdentifiers_vector &spec_Identifiers,
					 bool                            string = false,
					 int                             weight = 0);
	virtual void cast_Disjunction(Glucose::Solver                *solver,
				     SpecifiedBitIdentifiers_vector &spec_Identifiers,
				     int                             weight = 0);	

	virtual int count_ConditionalAllMutexConstraint(int                            &aux_Variable_cnt,
							int                            &total_Literal_cnt,
							const sSpecifiedBitIdentifier  &spec_condition,
							SpecifiedBitIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_ConditionalAllMutexConstraint(FILE                           *fw,
							   const sSpecifiedBitIdentifier  &spec_condition,
							   SpecifiedBitIdentifiers_vector &spec_Identifiers,
							   bool                            string = false,
							   int                             weight = 0);
	virtual void cast_ConditionalAllMutexConstraint(Glucose::Solver                *solver,
						       const sSpecifiedBitIdentifier  &spec_condition,
						       SpecifiedBitIdentifiers_vector &spec_Identifiers,
						       int                             weight = 0);	

	virtual int count_BitSet(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedBitIdentifier &spec_identifier) const;
	virtual int generate_BitSet(FILE *fw, const sSpecifiedBitIdentifier &spec_identifier, bool string = false, int weight = 0);
	virtual void cast_BitSet(Glucose::Solver *solver, const sSpecifiedBitIdentifier &spec_identifier, int weight = 0);	

	virtual int count_BitUnset(int &aux_Variable_cnt, int &total_Literal_cnt, const sSpecifiedBitIdentifier &spec_identifier) const;
	virtual int generate_BitUnset(FILE *fw, const sSpecifiedBitIdentifier &spec_identifier, bool string = false, int weight = 0);
	virtual void cast_BitUnset(Glucose::Solver *solver, const sSpecifiedBitIdentifier &spec_identifier, int weight = 0);	

	virtual int count_TriangleMutex(int                           &aux_Variable_cnt,
					int                           &total_Literal_cnt,
					const sSpecifiedBitIdentifier &spec_identifier_A,
					const sSpecifiedBitIdentifier &spec_identifier_B,
					const sSpecifiedBitIdentifier &spec_identifier_C) const;
	virtual int generate_TriangleMutex(FILE                          *fw,
					   const sSpecifiedBitIdentifier &spec_identifier_A,
					   const sSpecifiedBitIdentifier &spec_identifier_B,
					   const sSpecifiedBitIdentifier &spec_identifier_C,
					   bool                           string = false,
					   int                            weight = 0);
	virtual void cast_TriangleMutex(Glucose::Solver               *solver,
					const sSpecifiedBitIdentifier &spec_identifier_A,
					const sSpecifiedBitIdentifier &spec_identifier_B,
					const sSpecifiedBitIdentifier &spec_identifier_C,
					int                            weight = 0);

	virtual int count_MultiTriangleMutex(int                            &aux_Variable_cnt,
					     int                            &total_Literal_cnt,
					     const sSpecifiedBitIdentifier  &spec_identifier_A,
					     const sSpecifiedBitIdentifier  &spec_identifier_B,
					     SpecifiedBitIdentifiers_vector &spec_Identifiers_C) const;
	virtual int generate_MultiTriangleMutex(FILE                           *fw,
						const sSpecifiedBitIdentifier  &spec_identifier_A,
						const sSpecifiedBitIdentifier  &spec_identifier_B,
						SpecifiedBitIdentifiers_vector &spec_Identifiers_C,
						bool                            string = false,
						int                             weight = 0);
	virtual void cast_MultiTriangleMutex(Glucose::Solver                *solver,
					     const sSpecifiedBitIdentifier  &spec_identifier_A,
					     const sSpecifiedBitIdentifier  &spec_identifier_B,
					     SpecifiedBitIdentifiers_vector &spec_Identifiers_C,
					     int                             weight = 0);		

	virtual int count_BiangleMutex(int                           &aux_Variable_cnt,
				       int                           &total_Literal_cnt,
				       const sSpecifiedBitIdentifier &spec_identifier_A,
				       const sSpecifiedBitIdentifier &spec_identifier_B) const;
	virtual int generate_BiangleMutex(FILE                          *fw,
					  const sSpecifiedBitIdentifier &spec_identifier_A,
					  const sSpecifiedBitIdentifier &spec_identifier_B,
					  bool                           string = false,
					  int                            weight = 0);
	virtual void cast_BiangleMutex(Glucose::Solver               *solver,
				       const sSpecifiedBitIdentifier &spec_identifier_A,
				       const sSpecifiedBitIdentifier &spec_identifier_B,
				       int                            weight = 0);

	virtual int count_MultiBiangleMutex(int                            &aux_Variable_cnt,
					    int                            &total_Literal_cnt,
					    const sSpecifiedBitIdentifier  &spec_identifier_A,
					    SpecifiedBitIdentifiers_vector &spec_Identifiers_B) const;
	virtual int generate_MultiBiangleMutex(FILE                           *fw,
					       const sSpecifiedBitIdentifier  &spec_identifier_A,
					       SpecifiedBitIdentifiers_vector &spec_Identifiers_B,
					       bool                            string = false,
					       int                             weight = 0);
	virtual void cast_MultiBiangleMutex(Glucose::Solver                *solver,
					    const sSpecifiedBitIdentifier  &spec_identifier_A,
					    SpecifiedBitIdentifiers_vector &spec_Identifiers_B,
					    int                             weight = 0);		

	virtual int count_Implication(int                           &aux_Variable_cnt,
				      int                           &total_Literal_cnt,
				      const sSpecifiedBitIdentifier &spec_identifier_PREC,
				      const sSpecifiedBitIdentifier &spec_identifier_POST) const;
	virtual int generate_Implication(FILE                          *fw,
					 const sSpecifiedBitIdentifier &spec_identifier_PREC,
					 const sSpecifiedBitIdentifier &spec_identifier_POST,
					 bool                           string = false,
					 int                            weight = 0);
	virtual void cast_Implication(Glucose::Solver               *solver,
				      const sSpecifiedBitIdentifier &spec_identifier_PREC,
				      const sSpecifiedBitIdentifier &spec_identifier_POST,
				      int                            weight = 0);

	virtual int count_Bimplication(int                           &aux_Variable_cnt,
				       int                           &total_Literal_cnt,
				       const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
				       const sSpecifiedBitIdentifier &spec_identifier_PREC_B,				       
				       const sSpecifiedBitIdentifier &spec_identifier_POST) const;
	virtual int generate_Bimplication(FILE                          *fw,
					  const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
					  const sSpecifiedBitIdentifier &spec_identifier_PREC_B,					  
					  const sSpecifiedBitIdentifier &spec_identifier_POST,
					  bool                           string = false,
					  int                            weight = 0);
	virtual void cast_Bimplication(Glucose::Solver               *solver,
				       const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
				       const sSpecifiedBitIdentifier &spec_identifier_PREC_B,				       
				       const sSpecifiedBitIdentifier &spec_identifier_POST,
				       int                            weight = 0);		

	virtual int count_Implication(int                           &aux_Variable_cnt,
				      int                           &total_Literal_cnt,
				      const sSpecifiedBitIdentifier &spec_identifier_PREC,
				      const sSpecifiedBitIdentifier &spec_identifier_POST_A,
				      const sSpecifiedBitIdentifier &spec_identifier_POST_B) const;
	virtual int generate_Implication(FILE                          *fw,
					 const sSpecifiedBitIdentifier &spec_identifier_PREC,
					 const sSpecifiedBitIdentifier &spec_identifier_POST_A,
					 const sSpecifiedBitIdentifier &spec_identifier_POST_B,
					 bool                           string = false,
					 int                            weight = 0);
	virtual void cast_Implication(Glucose::Solver               *solver,
				      const sSpecifiedBitIdentifier &spec_identifier_PREC,
				      const sSpecifiedBitIdentifier &spec_identifier_POST_A,
				      const sSpecifiedBitIdentifier &spec_identifier_POST_B,
				      int                            weight = 0);	

	virtual int count_NonzeroImplication(int                             &aux_Variable_cnt,
					     int                             &total_Literal_cnt,
					     const sSpecifiedStateIdentifier &spec_identifier_PREC,
					     const sSpecifiedBitIdentifier   &spec_identifier_POST) const;
	virtual int generate_NonzeroImplication(FILE                            *fw,
						const sSpecifiedStateIdentifier &spec_identifier_PREC,
						const sSpecifiedBitIdentifier   &spec_identifier_POST,
						bool                             string = false,
						int                              weight = 0);
	virtual void cast_NonzeroImplication(Glucose::Solver                 *solver,
					     const sSpecifiedStateIdentifier &spec_identifier_PREC,
					     const sSpecifiedBitIdentifier   &spec_identifier_POST,
					     int                              weight = 0);	

	virtual int count_ZeroImplication(int                             &aux_Variable_cnt,
					  int                             &total_Literal_cnt,
					  const sSpecifiedBitIdentifier   &spec_identifier_PREC,
					  const sSpecifiedStateIdentifier &spec_identifier_POST) const;
	virtual int generate_ZeroImplication(FILE                            *fw,
					     const sSpecifiedBitIdentifier   &spec_identifier_PREC,
					     const sSpecifiedStateIdentifier &spec_identifier_POST,
					     bool                             string = false,
					     int                              weight = 0);
	virtual void cast_ZeroImplication(Glucose::Solver                 *solver,
					  const sSpecifiedBitIdentifier   &spec_identifier_PREC,
					  const sSpecifiedStateIdentifier &spec_identifier_POST,
					  int                              weight = 0);	

	virtual int count_Effect(int                           &aux_Variable_cnt,
				 int                           &total_Literal_cnt,
				 const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
				 const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
				 const sSpecifiedBitIdentifier &spec_identifier_POST) const;
	virtual int generate_Effect(FILE                          *fw,
				    const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
				    const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
				    const sSpecifiedBitIdentifier &spec_identifier_POST,
				    bool                           string = false,
				    int                            weight = 0);
	virtual void cast_Effect(Glucose::Solver               *solver,
				 const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
				 const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
				 const sSpecifiedBitIdentifier &spec_identifier_POST,
				 int                            weight = 0);	

	virtual int count_MultiImplication(int                            &aux_Variable_cnt,
					   int                            &total_Literal_cnt,
					   const sSpecifiedBitIdentifier  &spec_identifier_PREC,
					   SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const;
	virtual int generate_MultiImplication(FILE                           *fw,
					      const sSpecifiedBitIdentifier  &spec_identifier_PREC,
					      SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
					      bool                            string = false,
					      int                             weight = 0);
	virtual void cast_MultiImplication(Glucose::Solver                *solver,
					   const sSpecifiedBitIdentifier  &spec_identifier_PREC,
					   SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
					   int                             weight = 0);

	virtual int count_MultiImpliedImplication(int                            &aux_Variable_cnt,
						  int                            &total_Literal_cnt,
						  const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers_MIDDLE,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const;
	virtual int generate_MultiImpliedImplication(FILE                           *fw,
						     const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						     SpecifiedBitIdentifiers_vector &spec_Identifiers_MIDDLE,
						     SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						     bool                            string = false,
						     int                             weight = 0);
	virtual void cast_MultiImpliedImplication(Glucose::Solver                *solver,
						  const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers_MIDDLE,
						  SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						  int                             weight = 0);	

	virtual int count_MultiConjunctiveImplication(int                            &aux_Variable_cnt,
						      int                            &total_Literal_cnt,
						      const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const;
	virtual int generate_MultiConjunctiveImplication(FILE                           *fw,
							 const sSpecifiedBitIdentifier  &spec_identifier_PREC,
							 SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
							 bool                            string = false,
							 int                             weight = 0);
	virtual void cast_MultiConjunctiveImplication(Glucose::Solver                *solver,
						      const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						      int                             weight = 0);		
	
	virtual int count_MultiNegation(int                            &aux_Variable_cnt,
					int                            &total_Literal_cnt,
					SpecifiedBitIdentifiers_vector &spec_Identifiers) const;
	virtual int generate_MultiNegation(FILE                           *fw,
					   SpecifiedBitIdentifiers_vector &spec_Identifiers,
					   bool                            string = false,
					   int                             weight = 0);
	virtual void cast_MultiNegation(Glucose::Solver                *solver,
					SpecifiedBitIdentifiers_vector &spec_Identifiers,
					int                             weight = 0);	

	virtual int count_MultiNegativeImplication(int                            &aux_Variable_cnt,
						   int                            &total_Literal_cnt,
						   const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						   SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const;
	virtual int generate_MultiNegativeImplication(FILE                           *fw,
						      const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						      SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						      bool                            string = false,
						      int                             weight = 0);
	virtual void cast_MultiNegativeImplication(Glucose::Solver                *solver,
						   const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						   SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						   int                             weight = 0);	

	virtual int count_MultiExclusiveImplication(int                            &aux_Variable_cnt,
						    int                            &total_Literal_cnt,
						    const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const;
	virtual int generate_MultiExclusiveImplication(FILE                           *fw,
						       const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						       SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						       bool                            string = false,
						       int                             weight = 0);
	virtual void cast_MultiExclusiveImplication(Glucose::Solver                *solver,
						    const sSpecifiedBitIdentifier  &spec_identifier_PREC,
						    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
						    int                             weight = 0);	

	virtual int count_ConditionalEquality(int                             &aux_Variable_cnt,
					      int                             &total_Literal_cnt,
					      const sSpecifiedBitIdentifier   &spec_identifier_PREC,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B) const;
	virtual int generate_ConditionalEquality(FILE                            *fw,
						 const sSpecifiedBitIdentifier   &spec_identifier_PREC,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
						 const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
						 bool                             string = false,
						 int                              weight = 0);
	virtual void cast_ConditionalEquality(Glucose::Solver                 *solver,
					      const sSpecifiedBitIdentifier   &spec_identifier_PREC,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_A,
					      const sSpecifiedStateIdentifier &spec_identifier_THEN_B,
					      int                              weight = 0);	

	virtual int count_SwapConstraint(int                           &aux_Variable_cnt,
					 int                           &total_Literal_cnt,
					 const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
					 const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
					 const sSpecifiedBitIdentifier &spec_identifier_POST_A,
					 const sSpecifiedBitIdentifier &spec_identifier_POST_B) const;
	virtual int generate_SwapConstraint(FILE                          *fw,
					    const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
					    const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
					    const sSpecifiedBitIdentifier &spec_identifier_POST_A,
					    const sSpecifiedBitIdentifier &spec_identifier_POST_B,
					    bool                           string = false,
					    int                            weight = 0);
	virtual void cast_SwapConstraint(Glucose::Solver               *solver,
					 const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
					 const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
					 const sSpecifiedBitIdentifier &spec_identifier_POST_A,
					 const sSpecifiedBitIdentifier &spec_identifier_POST_B,
					 int                            weight = 0);	
	
	virtual int count_NegativeSwapConstraint(int                           &aux_Variable_cnt,
						 int                           &total_Literal_cnt,
						 const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						 const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
						 const sSpecifiedBitIdentifier &spec_identifier_POST_A,
						 const sSpecifiedBitIdentifier &spec_identifier_POST_B) const;
	virtual int generate_NegativeSwapConstraint(FILE                          *fw,
						    const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						    const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
						    const sSpecifiedBitIdentifier &spec_identifier_POST_A,
						    const sSpecifiedBitIdentifier &spec_identifier_POST_B,
						    bool                           string = false,
						    int                            weight = 0);
	virtual void cast_NegativeSwapConstraint(Glucose::Solver               *solver,
						 const sSpecifiedBitIdentifier &spec_identifier_PREC_A,
						 const sSpecifiedBitIdentifier &spec_identifier_PREC_B,
						 const sSpecifiedBitIdentifier &spec_identifier_POST_A,
						 const sSpecifiedBitIdentifier &spec_identifier_POST_B,
						 int                            weight = 0);	

	virtual int count_SwapConstraint(int                            &aux_Variable_cnt,
					 int                            &total_Literal_cnt,
					 const sSpecifiedBitIdentifier  &spec_identifier_PREC_A,
					 const sSpecifiedBitIdentifier  &spec_identifier_PREC_B,
					 SpecifiedBitIdentifiers_vector &spec_Identifiers_POST) const;
	virtual int generate_SwapConstraint(FILE                           *fw,
					    const sSpecifiedBitIdentifier  &spec_identifier_PREC_A,
					    const sSpecifiedBitIdentifier  &spec_identifier_PREC_B,
					    SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
					    bool                            string = false,
					    int                             weight = 0);
	virtual void cast_SwapConstraint(Glucose::Solver                *solver,
					 const sSpecifiedBitIdentifier  &spec_identifier_PREC_A,
					 const sSpecifiedBitIdentifier  &spec_identifier_PREC_B,
					 SpecifiedBitIdentifiers_vector &spec_Identifiers_POST,
					 int                             weight = 0);	

	virtual int count_Cardinality(int                            &aux_Variable_cnt,
				      int                            &total_Literal_cnt,
				      SpecifiedBitIdentifiers_vector &spec_Identifiers,
				      int                             cardinality) const;
	virtual int generate_Cardinality(FILE                           *fw,
					 SpecifiedBitIdentifiers_vector &spec_Identifiers,
					 int                             cardinality,
					 bool                            string = false,
					 int                             weight = 0);
	virtual void cast_Cardinality(Glucose::Solver                *solver,
				      SpecifiedBitIdentifiers_vector &spec_Identifiers,
				      int                             cardinality,
				      int                             weight = 0);

	/*----------------------------------------------------------------*/
	
	void cast_Clause(Glucose::Solver *solver, int lit_1);
	void cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2);
	void cast_Clause(Glucose::Solver *solver, int lit_1, int lit_2, int lit_3);
	void cast_Clause(Glucose::Solver *solver, std::vector<int> &Lits);	

    protected:
	sVariableStore_CNF *m_variable_store;

	int m_aux_Identifier_cnt;
	AuxiliaryIdentifiers_map m_auxiliary_Identifiers;
	AuxiliaryBitIdentifiers_map m_auxiliary_bit_Identifiers;
    };


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __CNF_H__ */
