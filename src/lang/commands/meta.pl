:- module(meta_commands, []).

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).
:- use_module(library('db/mongo/client'),
		[ mng_strip/4 ]).
:- use_module(library('lang/compiler')).
:- use_module(library('lang/query')).

%%%% query commands
:- query_compiler:add_command(call,   [ask,tell]).
:- query_compiler:add_command(once,   [ask,tell]).
:- query_compiler:add_command(limit,  [ask,tell]).
:- query_compiler:add_command(ignore, [ask,tell]).

%%%% query expansion

%% once(:Goal)
% Make a possibly nondet goal semidet, i.e., succeed at most once.
% TODO: triple has special handling for this (modifier), remove it?
%
query_compiler:step_expand(
		once(Goal), Expanded, Context) :-
	query_expand((call(Goal), !), Expanded, Context).

%% ignore(:Goal)
% Calls Goal as once/1, but succeeds, regardless of whether Goal succeeded or not.
% TODO: triple has special handling for this (modifier), remove it?
%
query_compiler:step_expand(
		ignore(Goal), Expanded, Context) :-
	query_expand(((call(Goal), !) ; true), Expanded, Context).

%%
query_compiler:step_expand(
		limit(Count, Goal),
		limit(Count, Expanded),
		Context) :-
	query_expand(Goal, Expanded, Context).

%%
query_compiler:step_var(limit(_Count, Terminals), Var) :-
	member(X,Terminals),
	query_compiler:step_var(X, Var).

query_compiler:step_var(limit(Count, _Terminals), Var) :-
	query_compiler:get_var([Count], Var).

query_compiler:step_var(call(Terminals, _Scope), Var) :-
	member(X,Terminals),
	query_compiler:step_var(X, Var).

query_compiler:step_var(call(_Terminals, Scope), Var) :-
	time_scope(Scope, Since, Until),
	member(X, [Since,Until]),
	mng_strip(X, _Operator, _Type, Y),
	query_compiler:step_var(Y, Var).

%% limit(+Count, :Goal)
% Limit the number of solutions.
% True if Goal is true, returning at most Count solutions.
%
query_compiler:step_compile(
		limit(Count, Terminals),
		Context, Pipeline) :-
	query_compiler:var_key_or_val(Count,Count0),
	% appended to inner pipeline of lookup
	Prefix=[],
	Suffix=[['$limit',Count0]],
	% create a lookup and append $limit to inner pipeline,
	% then unwind next and assign variables to the toplevel document.
	findall(Step,
		query_compiler:lookup_next_unwind(Terminals,
			Prefix, Suffix, Context, Step),
		Pipeline).

%%
query_compiler:step_compile(
		call(Terminals, Scope0),
		Context0,
		Pipeline) :-
	% get since/until values
	time_scope(Since0, Until0, Scope0),
	time_scope(Since1, Until1, Scope1),
	query_compiler:var_key_or_val(Since0,Since1),
	query_compiler:var_key_or_val(Until0,Until1),
	% remove previous scope from context
	merge_options([scope(Scope1)], Context0, Context1),
	% finally compile called goal
	% and replace the scope in compile context
	query_compile(Terminals, Pipeline, Context1).

