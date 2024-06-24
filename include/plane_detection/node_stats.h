/*
 * @description: 
 * @param : 
 * @return: 
 */
#ifndef _NODE_STATS_H_
#define _NODE_STATS_H_
#include <assert.h>
#include <plane_detection/eig33sym.hpp>
#include <vector>
#include <plane_detection/point_type.h>
struct Stats {
		float sx = 0.0, sy = 0.0, sz = 0.0, //sum of x/y/z
			sxx = 0.0, syy = 0.0, szz = 0.0, //sum of xx/yy/zz
			sxy = 0.0, syz = 0.0, sxz = 0.0; //sum of xy/yz/xz
		int N = 0; //#points in this PlaneSeg

		Stats() : sx(0), sy(0), sz(0),
			sxx(0), syy(0), szz(0),
			sxy(0), syz(0), sxz(0), N(0) {}

		//merge from two other Stats
		Stats(const Stats& a, const Stats& b) :
		sx(a.sx+b.sx), sy(a.sy+b.sy), sz(a.sz+b.sz),
			sxx(a.sxx+b.sxx), syy(a.syy+b.syy), szz(a.szz+b.szz),
			sxy(a.sxy+b.sxy), syz(a.syz+b.syz), sxz(a.sxz+b.sxz), N(a.N+b.N) {}
		
		Stats(const Stats& a, const Stats& b, const Stats& c, const Stats& d):
		sx(a.sx + b.sx + c.sx + d.sx), sy(a.sy + b.sy + c.sy + d.sy), sz(a.sz + b.sz + c.sz + d.sz),
		sxx(a.sxx + b.sxx + c.sxx + d.sxx), syy(a.syy + b.syy + c.syy + d.syy), 
		szz(a.szz + b.szz + c.szz + d.szz), sxy(a.sxy + b.sxy + c.sxy + d.sxy), 
		syz(a.syz + b.syz + c.syz + d.syz), sxz(a.sxz + b.sxz + c.sxz + d.sxz), 
		N(a.N + b.N + c.N + d.N){}

		inline void clear() {
			sx=sy=sz=sxx=syy=szz=sxy=syz=sxz=0;
			N=0;
		}

		inline Stats & operator=(const Stats & other)
		{
			sx = other.sx; sy = other.sy; sz = other.sz;
			sxx = other.sxx; syy = other.syy; szz = other.szz;
			sxy = other.sxy; syz = other.syz; sxz = other.sxz;
			N = other.N;
			return *this;
		}

		//push a new point (x,y,z) into this Stats
		inline void push(const float x, const float y, const float z) {
			sx+=x; sy+=y; sz+=z;
			sxx+=x*x; syy+=y*y; szz+=z*z;
			sxy+=x*y; syz+=y*z; sxz+=x*z;
			++N;
		}

		inline void push(const Eigen::Vector3f & point)
		{
			push(point.x(), point.y(), point.z());
		}

		inline void push(const IndexPoints & ps)
		{
			for (auto & iter_point : ps)
			{
				push(iter_point.second);
			}
		}

		//push a new Stats into this Stats
		inline void push(const Stats& other) {
			sx+=other.sx; sy+=other.sy; sz+=other.sz;
			sxx+=other.sxx; syy+=other.syy; szz+=other.szz;
			sxy+=other.sxy; syz+=other.syz; sxz+=other.sxz;
			N+=other.N;
		}

		//caller is responsible to ensure (x,y,z) was collected in this stats
		inline void pop(const float x, const float y, const float z) {
			sx-=x; sy-=y; sz-=z;
			sxx-=x*x; syy-=y*y; szz-=z*z;
			sxy-=x*y; syz-=y*z; sxz-=x*z;
			--N;

			assert(N>=0);
		}

		//caller is responsible to ensure {other} were collected in this stats
		inline void pop(const Stats& other) {
			sx-=other.sx; sy-=other.sy; sz-=other.sz;
			sxx-=other.sxx; syy-=other.syy; szz-=other.szz;
			sxy-=other.sxy; syz-=other.syz; sxz-=other.sxz;
			N-=other.N;

			assert(N>=0);
		}

		/**
		*  \brief PCA-based plane fitting
		*  
		*  \param [out] center center of mass of the PlaneSeg
		*  \param [out] normal unit normal vector of the PlaneSeg (ensure normal.z>=0)
		*  \param [out] mse mean-square-error of the plane fitting
		*  \param [out] curvature defined as in pcl
		*/
		inline void compute(Eigen::Vector3f& center, Eigen::Vector3f& normal,
			float& mse, float& curvature) const
		{
			assert(N>=4);

			const float sc=((float)1.0)/this->N;//this->ids.size();
			// const double sc=((double)1.0)/this->N;//this->ids.size();
			//calc plane equation: center, normal and mse
			center[0]=sx*sc;
			center[1]=sy*sc;
			center[2]=sz*sc;
			float K[3][3] = {
				{sxx-sx*sx*sc,sxy-sx*sy*sc,sxz-sx*sz*sc},
				{           0,syy-sy*sy*sc,syz-sy*sz*sc},
				{           0,           0,szz-sz*sz*sc}
			};
			K[1][0]=K[0][1]; K[2][0]=K[0][2]; K[2][1]=K[1][2];
			float sv[3]={0,0,0};
			float V[3][3]={0};
			LA::eig33sym(K, sv, V); //!!! first eval is the least one
			//LA.svd33(K, sv, V);
			// 这里是修改过的地方，由于我们的点云是转到机器人世界坐标系下，所以保证法向量向上
			// 后续如果机器人行走在局部地图坐标系下时，也需要将地图在局部坐标系下，并且坐标系z轴与重力方向正好相反
			if(V[0][0]*center[0]+V[1][0]*center[1]+V[2][0]*center[2]<=0) {//enforce dot(normal,center)<00 so normal always points towards camera
				normal[0]=V[0][0];
				normal[1]=V[1][0];
				normal[2]=V[2][0];
			} else {
				normal[0]=-V[0][0];
				normal[1]=-V[1][0];
				normal[2]=-V[2][0];
			}

			// if (V[2][0]<0)
			// {
			// 	normal[0]=-V[0][0];
			// 	normal[1]=-V[1][0];
			// 	normal[2]=-V[2][0];
			// }
			// else
			// {
			// 	normal[0]=V[0][0];
			// 	normal[1]=V[1][0];
			// 	normal[2]=V[2][0];
			// }
			mse = sv[0]*sc;
			curvature=sv[0]/(sv[0]+sv[1]+sv[2]);
		}
	};					//member points' 1st & 2nd order statistics

#endif