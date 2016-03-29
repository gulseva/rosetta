// -*- mode:c++;tab-width:2;indent-tabs-mode:t;show-trailing-whitespace:t;rm-trailing-spaces:t -*-
// vi: set ts=2 noet:
//
// (c) Copyright Rosetta Commons Member Institutions.
// (c) This file is part of the Rosetta software suite and is made available under license.
// (c) The Rosetta software is developed by the contributing members of the Rosetta Commons.
// (c) For more information, see http://www.rosettacommons.org. Questions about this can be
// (c) addressed to University of Washington UW TechTransfer, email: license@u.washington.edu.

/// @file   binder/util.cpp
/// @brief  Various helper functions
/// @author Sergey Lyskov

#include <util.hpp>

#include <enum.hpp>
#include <class.hpp>

#include <clang/AST/ASTContext.h>
#include <clang/AST/ExprCXX.h>

//#include <experimental/filesystem>
#include <cstdlib>
#include <fstream>

using namespace llvm;
using namespace clang;
using std::string;
using std::vector;

namespace binder {


/// Split string using given separator
vector<string> split(string const &buffer, string const & separator)
{
	string line;
	vector<string> lines;

	for(uint i=0; i<buffer.size(); ++i) {
		if( buffer.compare(i, separator.size(), separator) ) line.push_back( buffer[i] );
		else {
			lines.push_back(line);
			line.resize(0);
		}
	}

	if( line.size() ) lines.push_back(line);

	return lines;
}


/// Replace all occurrences of string
string replace(string const &s, string const & from, string const &to)
{
	string r{s};
	size_t i = r.size();
	while( ( i = r.rfind(from, i) ) != string::npos) {
		r.replace(i, from.size(), to);
		if(i) --i; else break;
	}

	return r;
}


/// check if string begins with given prefix
bool begins_with(std::string const &source, std::string const &prefix)
{
	return !source.compare(0, prefix.size(), prefix);
}


string indent(string const &code, string const &indentation)
{
	auto lines = split(code);
	string r;

	for(auto & l : lines) r += indentation + l + '\n';

	return r;
}


/// Try to read exisitng file and if content does not match to code - write a new version. Also create nested dirs starting from prefix if nessesary.
void update_source_file(std::string const &prefix, std::string const &file_name, std::string const &code)
{
	string path = prefix;

	vector<string> dirs = split(file_name, "/");  dirs.pop_back();
	for(auto &d : dirs) path += "/" + d;

	//std::experimental::filesystem::create_directories(path);
	string command_line = "mkdir -p "+path;
	system( command_line.c_str() );

	string full_file_name = prefix + file_name;
	std::ifstream f(full_file_name);
	std::string old_code((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

	if( old_code != code ) std::ofstream(full_file_name) << code;
}


string namespace_from_named_decl(NamedDecl const *decl)
{
	string qn = decl->getQualifiedNameAsString();
	string n  = decl->getNameAsString();
	//name = decl->getQualifiedNameAsString();

	int namespace_len = qn.size() - n.size();

	string path = decl->getQualifiedNameAsString().substr(0, namespace_len > 1 ? namespace_len-2 : namespace_len );  // removing trailing '::'

	return path;
}


/// generate unique string representation of type represented by given declaration
string typename_from_type_decl(TypeDecl *decl)
{
	return decl->getTypeForDecl()->getCanonicalTypeInternal()/*getCanonicalType()*/.getAsString();
	//CanQualType 	getCanonicalTypeUnqualified () const
}


/// Calculate base (upper) namespace for given one: core::pose::motif --> core::pose
string base_namespace(string const & ns)
{
	size_t f = ns.rfind("::");
	if( f == string::npos ) return "";
	else return ns.substr(0, f);
}


/// Calculate last namespace for given one: core::pose::motif --> motif
string last_namespace(string const & ns)
{
	size_t f = ns.rfind("::");
	if( f == string::npos ) return ns;
	else return ns.substr(f+2, ns.size()-f-2);
}


// replace all _Bool types with bool
void fix_boolean_types(string &type)
{
	string B("_Bool");
	size_t i = 0;
	while( ( i = type.find(B, i) ) != string::npos ) {
		if( ( i==0  or ( !std::isalpha(type[i-1]) and  !std::isdigit(type[i-1]) ) ) and
			( i+B.size() == type.size()  or ( !std::isalpha(type[i+B.size()]) and  !std::isdigit(type[i+B.size()]) ) ) ) type.replace(i, B.size(), "bool");
		++i;
	}
}


// Generate string representation of given expression
string expresion_to_string(clang::Expr *e)
{
	clang::LangOptions lang_opts;
	lang_opts.CPlusPlus = true;
	clang::PrintingPolicy Policy(lang_opts);

	std::string _;
	llvm::raw_string_ostream s(_);
	e->printPretty(s, 0, Policy);
	return s.str();
}


// Generate string representation of given TemplateArgument
string template_argument_to_string(clang::TemplateArgument const &t)
{
	clang::LangOptions lang_opts;
	lang_opts.CPlusPlus = true;
	clang::PrintingPolicy Policy(lang_opts);

	std::string _;
	llvm::raw_string_ostream s(_);
	t.print(Policy, s);
	return s.str();
}


// calcualte line in source file for NamedDecl
string line_number(NamedDecl *decl)
{
	ASTContext & ast_context( decl->getASTContext() );
	SourceManager & sm( ast_context.getSourceManager() );

	return std::to_string( sm.getSpellingLineNumber(decl->getLocation() ) );
}

} // namespace binder