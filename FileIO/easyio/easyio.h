#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <list>
#include <vector>
#include <map>
#include "../../Math/Matrix.h"

#define STRING_UNINITIALIZED "nosep"

typedef std::vector<std::vector<std::string> > string_matrix2d;
typedef std::vector<std::string> string_matrix1d;
typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<double> matrix1d;




class FileManip{
public:
	static string_matrix2d read(std::string filename, std::string separator = STRING_UNINITIALIZED);
	static matrix2d readDouble(std::string filename, std::string separator = STRING_UNINITIALIZED);
	static string_matrix1d divide(std::string myString, std::string separator);
	static matrix2d str2double(string_matrix2d mystring);
};
/*
template <typename T>
void load_variable(std::vector<std::vector<T> >& var, std::string filename, std::string separator = STRING_UNINITIALIZED){
	string_matrix2d f = FileManip::read(filename, separator);

	ostringstream convert;
	for (int i=0; i<f.size(); i++){
		for (int j=0; j<f[i].size(); j++){
			int vardata = stoi(f[i][j]);
			var[i][j] = static_cast<T>(vardata);
		}
	}
}
*/

class DataManip{
public:
	template <typename T>
	static void load_variable(std::vector<std::vector<T> >* var, std::string filename, std::string separator = STRING_UNINITIALIZED){
		string_matrix2d f = FileManip::read(filename, separator);
		*var = std::vector<std::vector<T> >(f.size());

		std::stringstream convert;
		for (int i=0; i<f.size(); i++){
			var->at(i) = std::vector<T>(f[i].size());
			for (int j=0; j<f[i].size(); j++){
				convert.str(f[i][j]);
				T hello;
				convert >> hello;
				var->at(i)[j] = hello;
				//convert >> var->at(i)[j];
				convert.str("");
				convert.clear();
			}
		}
	}

	static matrix2d stringToDouble(string_matrix2d stringVector);
	static double stringToDouble(std::string s);
	static matrix1d getColumn(matrix2d doubleVector, int col);
	static matrix2d getColumns(matrix2d doubleVector, std::vector<int> cols);

	static std::map<std::string,double> readVariableFile(std::string fileName){
		string_matrix2d rawData = FileManip::read(fileName);
		std::map<std::string,double>  processedData;
		for (int i=0; i<rawData.size(); i++){
			processedData.insert(std::pair<std::string,double>(rawData[i][0],atof(rawData[i][1].c_str())));
		}
		return processedData;
	}

	static void fillVariable(double &var, std::string varname, std::map<std::string,double> varfilecontents){
		var = varfilecontents[varname];
	}

	static void fillVariable(int &var, std::string varname, std::map<std::string,double> varfilecontents){
		var = (int)varfilecontents[varname];
	}
};


class PrintOut{
public:
	static void screen(string_matrix1d myVector, std::string separator="\n");

	template<typename M1>
	static void toFile1D(M1 output1d, std::string fileName, std::string separator=","){
		/*
		Output function for 1d vector, list, etc: anything that can be iterated over
		*/
		std::ofstream file; // original: ofstream file;
		file.open(fileName.c_str());
		if (file.is_open()){
			fileOut1D(output1d,file,separator);
			file.close();
		}
		else {
			printf("Failed to open %s.",fileName.c_str());
		}
	}

	template<typename M2>
	static void toFile2D(M2 output2d, std::string fileName, std::string separator=","){
		/*
		Output function for 2d vector, list, etc: anything that can be iterated over
		*/
		std::ofstream file;
		file.open(fileName.c_str());
		if (file.is_open()){
			for (typename M2::iterator outer=output2d.begin(); outer!=output2d.end(); outer++){ // original: for (M2::iterator outer=output2d.begin(); outer!=output2d.end(); outer++){
				fileOut1D(*outer, file,separator);
				file << "\n";
			}
			file.close();
		}
		else {
			printf("Failed to open %s.",fileName.c_str());
		}
	}

	template<typename M2>
	static void toFileMatrix2D(Numeric_lib::Matrix<M2,2> output2d, std::string fileName, std::string separator=","){
		/*
		Output function for 2d vector, list, etc: anything that can be iterated over
		*/
		std::ofstream file;
		file.open(fileName.c_str());
		if (file.is_open()){
			for (int i=0; i<output2d.dim1(); i++){
				for (int j=0; j<output2d.dim2(); j++){
					file << output2d(i,j) << separator;
				}
				file << "\n";
			}
			file.close();
		}
		else {
			printf("Failed to open %s.",fileName.c_str());
		}
	}

	template <typename M3>
	static void toFile3D(M3 output3d, std::string fileName, std::string separator=","){
		/*
		Output function for 3d vector, list, etc: anything that can be iterated over
		*/
		std::ofstream file; // original: ofstream file;
		file.open(fileName.c_str());
		if (file.is_open()){
			for (typename M3::iterator outer=output3d.begin(); outer!=output3d.end(); outer++){ // original: for (M3::iterator outer=output3d.begin(); outer!=output3d.end(); outer++){
				fileOut2D(*outer, file, separator);
				file << "\n";
			}
			file.close();
		}
		else {
			printf("Failed to open %s.",fileName.c_str());
		}
	}

private:
	template <typename M1>
	static void fileOut1D(M1 output1d, std::ofstream &file, std::string separator){
		/*
		Helper function for 1- and 2-d file output
		*/
		for (typename M1::iterator outer=output1d.begin(); outer!=output1d.end(); outer++){ // original: for (M1::iterator outer=output1d.begin(); outer!=output1d.end(); outer++){
			file << *outer << separator;
		}
	}
	template <typename M2>
	static void fileOut2D(M2 output2d, std::ofstream &file, std::string separator){
		/*
		Helper function for 3-d file output
		*/
		for (typename M2::iterator outer=output2d.begin(); outer!=output2d.end(); outer++){ // original: for (M2::iterator outer=output2d.begin(); outer!=output2d.end(); outer++){
			fileOut1D(*outer, file, separator);
		}
	}

};
