#include <iostream>
#include <cmath>
using namespace std;

class vector3d{
    public:
        float x,y,z;

        vector3d (float x,float y,float z): x(x) , y(y) ,z(z) {}

        vector3d operator * (float scalar) const {

        return vector3d(x * scalar, y * scalar, z * scalar);
        }
        
        vector3d operator / (float scalar) const {
            
            return vector3d(x / scalar, y / scalar, z / scalar);
        } 
        
        vector3d operator + (vector3d vecA) const {

        return vector3d(x +vecA.x, y +vecA.y, z +vecA.z);
        }

         vector3d operator - (vector3d vecA) const {

        return vector3d(x -vecA.x, y -vecA.y, z -vecA.z);
        }

        float dot(vector3d vecA,vector3d vecB){
            return vecA.x*vecB.x +vecA.y*vecB.y+vecA.z*vecB.z;
        }
        vector3d operator * (vector3d vecA) const {
            
            return vector3d(vecA.z*y-vecA.y*z,vecA.x*z-vecA.z*x,vecA.y*x-y*vecA.x) ;
        } 
        float mod(vector3d vecA){
            return  sqrt(dot(vecA,vecA));
        }
        vector3d unit(vector3d vecA){
            return vector3d(x/mod(vecA),y/mod(vecA),z/mod(vecA));
        }
        vector3d component_multi(vector3d vecA,vector3d vecB){
            return vector3d(vecA.x*vecB.x,vecA.y*vecB.y,vecA.z*vecB.z);
        }

        float vec_STP(vector3d vecA,vector3d vecB,vector3d vecC){
            return dot(vecA,(vecB*vecC));
        }
        vector3d vec_VTP(vector3d vecA,vector3d vecB,vector3d vecC){
            return  vecA*(vecB*vecC);
        }
        float angle(vector3d vecA,vector3d vecB){
            float costheta=dot(vecA,vecB)/(mod(vecA)*mod(vecB));
          return acos(costheta);
        }
        vector3d projecVec(vector3d vecA,vector3d vecB){
            float scalar=dot(vecA,vecB)/dot(vecB,vecB);
            return vecB *scalar;
        }
        vector3d perpendicular_projecVec(vector3d vecA,vector3d vecB){
            vector3d vect=projecVec(vecA,vecB);
            return vecA-vect;
        }
        vector3d rotate_x(float theta) { 
            return vector3d(this->x, this->y * cos(theta) - this->z * sin(theta), this->y * sin(theta) + this->z * cos(theta));
        }
        vector3d rotate_y(float theta)  {
            return vector3d(this->x * cos(theta) + this->z * sin(theta), this->y, this->z * cos(theta) - this->x * sin(theta));
         }
        vector3d rotate_z(float theta)  {
            return vector3d(this->x * cos(theta) - this->y * sin(theta), this->x * sin(theta) + this->y * cos(theta), this->z);
        }
        vector3d VecGenerator(float R,float alpha,float beta,float gamma){
            return vector3d(R*cos(alpha),R*cos(beta),R*cos(gamma));
        }


        void print() const {
        cout << "Vector(" << x << ", " << y << ", " << z << ")" << endl;
    }
};



int main(){

    vector3d vecA(1,1,1);
    vector3d vecB(3,2,1);
    vector3d vecC=vecA.VecGenerator(sqrt(50),3.0/(5*sqrt(2)),4.0/(5*sqrt(2)),1.0/sqrt(2));

    
    cout<<"The given vectors are vecA :" ;
    cout<< "("<< vecA.x << ", " << vecA.y << ", " << vecA.z <<")" ", vecB :"  "("<< vecB.x << ", " << vecB.y << ", " << vecB.z <<")" " and vecC :" "("<< vecC.x << ", " << vecC.y << ", " << vecC.z <<")"<<endl;
    float scalar=1.2;
    vector3d scale_vecA=vecA * scalar;
    cout<<"Vector multiplied with scalar :";
    scale_vecA.print();

    vector3d div_vecA=vecA / scalar;
    cout<<"Vector divided by scalar :";
    div_vecA.print();

    vector3d vec_sum= vecA + vecB;
    cout<<"The sum of two vectors is :";
    vec_sum.print();

    vector3d vec_diff= vecA - vecB;
    cout<<"The difference between two vectors is :";
    vec_diff.print();

    cout<<"The modulus of the vectorA is :"<<vecA.mod(vecA)<<endl;

    int dot_prod= vecA.dot(vecA , vecB);
    cout<<"Dot product of two vectors is :"<<dot_prod<<endl;

    vector3d cross_prod= vecA * vecB;
    cout<<"Cross product of two vectors is :";
    cross_prod.print();

    vector3d unit_vec=vecA.unit(vecA);
    cout<<"The unit vector is :";
    unit_vec.print();

    vector3d compVec=vecC.component_multi(vecA,vecB);
    cout<<"The component multiplication of two vector results in :";
    compVec.print();

    float angle=vecC.angle(vecA,vecB);
    cout<<"The angle between vector A and B is :"<<angle<<endl;

    float vecSTP= vecA.vec_STP(vecA,vecB,vecC);
    cout<<"The scalar-triple product of the vector A,B and C: "<<vecSTP<<endl;

    vector3d vecVTP=vecA.vec_VTP(vecA,vecB,vecC);
    cout<<"The vector-triple product of the vector A,B and C :";
    vecVTP.print();

    vector3d proj=vecC.projecVec(vecA,vecB);
    cout<<"The projection vector of A on B is :";
    proj.print();
    vector3d perpproj=vecC.perpendicular_projecVec(vecA,vecB);
    cout<<"The perpendicular projection vector of A on B is :";
        perpproj.print();
    
    vector3d rotatedXvecA=vecA.rotate_x(0.52359877559829887307710723054658);
    cout<<"VecA rotated by 30degree about x is:";
    rotatedXvecA.print();

    vector3d rotatedYvecA=vecA.rotate_y(0.52359877559829887307710723054658);
    cout<<"VecA rotated by 30degree about y is:";
    rotatedYvecA.print();

    vector3d rotatedZvecA=vecA.rotate_x(0.52359877559829887307710723054658);
    cout<<"VecA rotated by 30degree about z is:";
    rotatedZvecA.print();
   

    return 0;
}