#include "CFremenGrid.h"

using namespace std;

CFremenGrid::CFremenGrid(float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize)
{
	oX = originX;
	oY = originY;
	oZ = originZ;
	xDim = dimX;
	yDim = dimY;
	zDim = dimZ;
	resolution = cellSize;
	numCells = xDim*yDim*zDim;
	aux = (char*) malloc(numCells*sizeof(char));
	probs = (float*) malloc(numCells*sizeof(float));
	for (int i = 0;i<numCells;i++)probs[i] = 0.5;
	if (debug) printf("Float size: %i \n",(int)sizeof(double));
	//cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
	//for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	
}

CFremenGrid::~CFremenGrid()
{
	//for (int i=0;i<numCells;i++) free(cellArray[i]);
	//free(cellArray);

}

int CFremenGrid::getIndex(float x,float  y,float  z)
{
	int iX,iY,iZ;
	iX = (int)((x-oX)/resolution);
	iY = (int)((y-oY)/resolution);
	iZ = (int)((z-oZ)/resolution);
	if (iX < xDim && iY < yDim && iZ < zDim && iX >= 0 && iY >=0 && iZ >= 0) return iX+xDim*(iY+yDim*iZ);
	return 0;
}

float CFremenGrid::getInformation(float sx,float sy,float sz,float phiRange,float psiRange,float range,float timeStamp)
{
	float x[200000];	
	float y[200000];	
	float z[200000];
	int size = 0;
	for (int phi = phiRange;phi<phiRange;phi+=0.01){
		for (int psi = -psiRange;psi<psiRange;psi+=0.01){
			x[size] = range*cos(phi)*cos(psi); 
			y[size] = range*sin(phi)*cos(psi); 
			z[size] = range*sin(psi); 
		}
		size++;
	}

	CTimer timer;
	timer.reset();
	timer.start();

	float px,py,pz,rx,ry,rz,ax,ay,az,bx,by,bz,cx,cy,cz;
	int startIndex,ix,iy,iz,index,final,xStep,yStep,zStep;
	unsigned char process[size];
	bool subsample = true;
	
	memset(aux,0,sizeof(char));
	//rescale ray intersections to match the grid
	//rescale only one ray per cell, adjust ray direction to go to the ray centre 
	memset(process,0,size);
	for (int i = 0;i<size+1;i++){
		x[i] = fmin(fmax(floor((x[i]-oX)/resolution),0)+0.5,xDim-1);
		y[i] = fmin(fmax(floor((y[i]-oY)/resolution),0)+0.5,yDim-1);
		z[i] = fmin(fmax(floor((z[i]-oZ)/resolution),0)+0.5,zDim-1);
		final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
		if (aux[final] != 1){
			aux[final] = 1;
			process[i] = 1;
		}
	}
	printf("Rescaling %i\n",timer.getTime());
	//raycast origin in float and int 
	int i = 0;
	//calculate the point of origin
	px = x[size];
	py = y[size];
	pz = z[size];
	//calculate the initial cell index
	startIndex =  (int)px+xDim*((int)py+yDim*((int)pz));
	int prepare  = 0;
	int calculate = 0;;

	for (int i = 0;i<size;i++)
	{
		timer.reset();
		if (process[i]){
			//calculate the ray vector 
			rx = (x[i]-px);
			ry = (y[i]-py);
			rz = (z[i]-pz);

			//calculate the general direction of the ray 
			ix = 1; 
			iy = 1; 
			iz = 1; 
			if (rx < 0) ix = -1;
			if (ry < 0) iy = -1;
			if (rz < 0) iz = -1;
			if (fabs(rx) < 1.0/xDim) ix = 0; 
			if (fabs(ry) < 1.0/yDim) iy = 0; 
			if (fabs(rz) < 1.0/zDim) iz = 0; 

			//establish increments when moving in the grid
			xStep = ix; 
			yStep = iy*xDim; 
			zStep = iz*xDim*yDim;

			//establish the first and last cell index 
			index = startIndex;
			final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
			//initialize values of the expected intersections
			bx=by=bz=cx=cy=cz = xDim*yDim*zDim; //high enough to be outside of the grid

			//initialize first intersection planes - these are in the general direction of the ray vector
			ax = floor(px)+(ix+1)/2;
			ay = floor(py)+(iy+1)/2;
			az = floor(pz)+(iz+1)/2;

			//calculate intersections with the planes
			if (ix != 0) bx = (ax-px)/rx;
			if (iy != 0) by = (ay-py)/ry;
			if (iz != 0) bz = (az-pz)/rz;

			//calculate the position increments when intersecting the following planes 
			if (ix != 0) cx = ix/rx;
			if (iy != 0) cy = iy/ry;
			if (iz != 0) cz = iz/rz;

			//if (debug) printf("Indices %i %i %.2f %.2f %.2f %.2f %.2f %.2f \n",index,final,bx,by,bz,cx,cy,cz);
			prepare += timer.getTime();
			timer.reset();
		
			// start the grid traversal process 
			for (int j=0;index!=final;j++)
			{
				//if (debug) printf("Index %06i %06i %06i %.2f %.2f %.2f %.2f\n",index,final,startIndex,bx,bx*rx+px,by*ry+py,bz*rz+pz);
				if (aux[index] == 0){
					aux[index] = 1;
					probs[index] = 0;
				}
				if (bx < by && bx < bz)
				{
					bx+=cx;
					index+=xStep;
				}
				else if (by < bz)
				{
					by+=cy;
					index+=yStep;
				}else{
					bz+=cz;
					index+=zStep;
				}
			}
			calculate += timer.getTime();
		}
	}
	printf("Times to prepare and calculate %i %i \n",prepare,calculate);
}

//ultrafast grid update 
void CFremenGrid::incorporate(float *x,float *y,float *z,int size)
{
	CTimer timer;
	timer.reset();
	timer.start();

	float px,py,pz,rx,ry,rz,ax,ay,az,bx,by,bz,cx,cy,cz;
	int startIndex,ix,iy,iz,index,final,xStep,yStep,zStep;
	unsigned char process[size];
	bool subsample = true;
	
	memset(aux,0,sizeof(char));
	//rescale ray intersections to match the grid
	if (subsample == false){
		//rescale all
		for (int i = 0;i<size+1;i++)
		{
			x[i] = fmin(fmax((x[i]-oX)/resolution,0.5),xDim-1);
			y[i] = fmin(fmax((y[i]-oY)/resolution,0.5),yDim-1);
			z[i] = fmin(fmax((z[i]-oZ)/resolution,0.5),zDim-1);
		}
		memset(process,1,size);
	}else{
		//rescale only one ray per cell, adjust ray direction to go to the ray centre 
		memset(process,0,size);
		for (int i = 0;i<size+1;i++){
			x[i] = fmin(fmax(floor((x[i]-oX)/resolution),0)+0.5,xDim-1);
			y[i] = fmin(fmax(floor((y[i]-oY)/resolution),0)+0.5,yDim-1);
			z[i] = fmin(fmax(floor((z[i]-oZ)/resolution),0)+0.5,zDim-1);
			final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
			if (aux[final] != 1){
				aux[final] = 1;
				probs[final] = 1;
				process[i] = 1;
			}
		}
	}
	printf("Rescaling %i\n",timer.getTime());
	//raycast origin in float and int 
	int i = 0;
	//calculate the point of origin
	px = x[size];
	py = y[size];
	pz = z[size];
	//calculate the initial cell index
	startIndex =  (int)px+xDim*((int)py+yDim*((int)pz));
	int prepare  = 0;
	int calculate = 0;;

	for (int i = 0;i<size;i++)
	{
		timer.reset();
		if (process[i]){
			//calculate the ray vector 
			rx = (x[i]-px);
			ry = (y[i]-py);
			rz = (z[i]-pz);

			//calculate the general direction of the ray 
			ix = 1; 
			iy = 1; 
			iz = 1; 
			if (rx < 0) ix = -1;
			if (ry < 0) iy = -1;
			if (rz < 0) iz = -1;
			if (fabs(rx) < 1.0/xDim) ix = 0; 
			if (fabs(ry) < 1.0/yDim) iy = 0; 
			if (fabs(rz) < 1.0/zDim) iz = 0; 

			//establish increments when moving in the grid
			xStep = ix; 
			yStep = iy*xDim; 
			zStep = iz*xDim*yDim;

			//establish the first and last cell index 
			index = startIndex;
			final = (int)x[i]+xDim*((int)y[i]+yDim*((int)z[i]));
			//initialize values of the expected intersections
			bx=by=bz=cx=cy=cz = xDim*yDim*zDim; //high enough to be outside of the grid

			//initialize first intersection planes - these are in the general direction of the ray vector
			ax = floor(px)+(ix+1)/2;
			ay = floor(py)+(iy+1)/2;
			az = floor(pz)+(iz+1)/2;

			//calculate intersections with the planes
			if (ix != 0) bx = (ax-px)/rx;
			if (iy != 0) by = (ay-py)/ry;
			if (iz != 0) bz = (az-pz)/rz;

			//calculate the position increments when intersecting the following planes 
			if (ix != 0) cx = ix/rx;
			if (iy != 0) cy = iy/ry;
			if (iz != 0) cz = iz/rz;

			//if (debug) printf("Indices %i %i %.2f %.2f %.2f %.2f %.2f %.2f \n",index,final,bx,by,bz,cx,cy,cz);
			prepare += timer.getTime();
			timer.reset();
		
			// start the grid traversal process 
			for (int j=0;index!=final;j++)
			{
				//if (debug) printf("Index %06i %06i %06i %.2f %.2f %.2f %.2f\n",index,final,startIndex,bx,bx*rx+px,by*ry+py,bz*rz+pz);
				if (aux[index] == 0){
					aux[index] = 1;
					probs[index] = 0;
				}
				if (bx < by && bx < bz)
				{
					bx+=cx;
					index+=xStep;
				}
				else if (by < bz)
				{
					by+=cy;
					index+=yStep;
				}else{
					bz+=cz;
					index+=zStep;
				}
			}
			calculate += timer.getTime();
		}
	}
	printf("Times to prepare and calculate %i %i \n",prepare,calculate);
}

void CFremenGrid::save(const char* filename,bool lossy,int forceOrder)
{
	FILE* f=fopen(filename,"w");
	fwrite(&xDim,sizeof(int),1,f);
	fwrite(&yDim,sizeof(int),1,f);
	fwrite(&zDim,sizeof(int),1,f);
	fwrite(&oX,sizeof(float),1,f);
	fwrite(&oY,sizeof(float),1,f);
	fwrite(&oZ,sizeof(float),1,f);
	fwrite(&resolution,sizeof(float),1,f);
	fwrite(probs,sizeof(float),numCells,f);
//	for (int i=0;i<numCells;i++) cellArray[i]->save(f,lossy,forceOrder);
	fclose(f);
}

bool CFremenGrid::load(const char* filename)
{
	int ret = 0;
	FILE* f=fopen(filename,"r");
	if (f == NULL){
		printf("FrOctomap %s not found, aborting load.\n",filename);
		return false;
	}
/*	printf("Loading FrOctomap %s.\n",filename);
	for (int i=0;i<numCells;i++){
		 free(cellArray[i]);
	}
	free(cellArray);*/
	ret = fread(&xDim,sizeof(int),1,f);
	ret = fread(&yDim,sizeof(int),1,f);
	ret = fread(&zDim,sizeof(int),1,f);
	ret = fread(&oX,sizeof(float),1,f);
	ret = fread(&oY,sizeof(float),1,f);
	ret = fread(&oZ,sizeof(float),1,f);
	ret = fread(&resolution,sizeof(float),1,f);
	numCells = xDim*yDim*zDim;
	ret = fread(probs,sizeof(float),numCells,f);
	/*cellArray = (CFrelement**) malloc(numCells*sizeof(CFrelement*));
	for (int i=0;i<numCells;i++) cellArray[i] = new CFrelement();
	for (int i=0;i<numCells;i++){
		cellArray[i]->load(f);
		cellArray[i]->signalLength = signalLength;
	}
	printf("FrOctomap %s loaded: name %s with %i: %ix%ix%i cells and %i observations.\n",filename,name,numCells,xDim,yDim,zDim,signalLength);
	 */
	fclose(f);
	return true;
}

void CFremenGrid::print(bool verbose)
{
/*	for (int i = 0;i<numCells;i++){
		if (cellArray[i]->order > 0 || cellArray[i]->outliers > 0){
			printf("Cell: %i ",i);
			cellArray[i]->print(verbose);
		}
	}*/
}

float CFremenGrid::estimate(unsigned int index,float timeStamp)
{
	return probs[index];
}

float CFremenGrid::estimate(float x,float y,float z,float t)
{

}

