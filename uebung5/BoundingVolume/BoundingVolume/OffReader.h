#include "vecmath.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


/// Läd ein OFF-File as_FileName und speichert alle Knoten in ak_Vertices
//* Die Indizierung der Flächen wirdin ak_Indices abgelegt.

int LoadOffFile(char* as_FileName, std::vector<Vector3d>& ak_Vertices, std::vector<int>& ak_Indices) 
{
	std::cout << "LoadOffFile(\"" << as_FileName << "\");" << std::endl;

	std::vector<float> lk_Faces;
	int li_TriangleCounter = 0;
	int li_VerticeLength = 0;
	int li_FaceCounter = 0;

	std::ifstream lk_InStream(as_FileName);
	if(!lk_InStream)
	{
		std::cout << "Off-Datei nicht gefunden!" << std::endl;
		return -1;
	}
	std::string ls_FileHeader;
	std::getline(lk_InStream, ls_FileHeader);
	if (ls_FileHeader.length() > 0 && ls_FileHeader[ls_FileHeader.length()-1] == 0xd) 
				ls_FileHeader.erase (ls_FileHeader.length()-1,1);

	if(ls_FileHeader.compare ("OFF") == 0)
	{
		std::cout << "Lese OFF-Datei..." << std::endl;
	}
	else 
	{
		std::cout << "Keine gueltige OFF-Datei!" << std::endl;
		return -1;
	}

	lk_InStream >> li_VerticeLength;
	std::cout << "Knoten: " << li_VerticeLength << std::endl;
	lk_InStream >> li_FaceCounter;
	std::cout << "Flaechen: " << li_FaceCounter << std::endl;
	int li_Edges;
	lk_InStream >> li_Edges;
	std::cout << "Kanten: " << li_Edges << std::endl;

	for(int i = 0; i < li_VerticeLength; i++) 
	{
		float x, y, z;
		lk_InStream >> x;
		lk_InStream >> y;
		lk_InStream >> z;
		ak_Vertices.push_back(Vector3d(x, y, z));
	}

	int li_Temp1, li_Temp2, li_Temp3;

	for (int i = 0; i < li_FaceCounter; i++) 
	{
		int li_Kanten;
		lk_InStream >> li_Kanten;
		for(int j = 0; j < li_Kanten - 2; j++)
		{
			if (j == 0)
			{
				lk_InStream >> li_Temp1;
				lk_InStream >> li_Temp2;
				lk_InStream >> li_Temp3;
			}
			if (j > 0)
			{
				li_Temp2 = li_Temp3;
				lk_InStream >> li_Temp3;
			}
			li_TriangleCounter++;
			lk_Faces.push_back(float(li_Temp1));
			lk_Faces.push_back(float(li_Temp2));
			lk_Faces.push_back(float(li_Temp3));
		}
	}

	for (int i = 0; i < li_TriangleCounter * 3; i++)
	{
		ak_Indices.push_back(int(lk_Faces[i]));
	}

	//OUT: lk_Vertices, lk_Indices
	return 0;
}




