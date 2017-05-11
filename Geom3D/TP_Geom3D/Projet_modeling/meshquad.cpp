#include "meshquad.h"
#include "matrices.h"

MeshQuad::MeshQuad():
	m_nb_ind_edges(0)
{

}


void MeshQuad::gl_init()
{
	m_shader_flat = new ShaderProgramFlat();
	m_shader_color = new ShaderProgramColor();

	//VBO
	glGenBuffers(1, &m_vbo);

	//VAO
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_flat->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_flat->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &m_vao2);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_color->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_color->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);


	//EBO indices
	glGenBuffers(1, &m_ebo);
	glGenBuffers(1, &m_ebo2);
}

void MeshQuad::gl_update()
{
	//VBO
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_points.size() * sizeof(GLfloat), &(m_points[0][0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	std::vector<int> tri_indices;
	convert_quads_to_tris(m_quad_indices,tri_indices);

	//EBO indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,tri_indices.size() * sizeof(int), &(tri_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


	std::vector<int> edge_indices;
	convert_quads_to_edges(m_quad_indices,edge_indices);
	m_nb_ind_edges = edge_indices.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,m_nb_ind_edges * sizeof(int), &(edge_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}



void MeshQuad::set_matrices(const Mat4& view, const Mat4& projection)
{
	viewMatrix = view;
	projectionMatrix = projection;
}

void MeshQuad::draw(const Vec3& color)
{

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);

	m_shader_flat->startUseProgram();
	m_shader_flat->sendViewMatrix(viewMatrix);
	m_shader_flat->sendProjectionMatrix(projectionMatrix);
	glUniform3fv(m_shader_flat->idOfColorUniform, 1, glm::value_ptr(color));
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo);
	glDrawElements(GL_TRIANGLES, 3*m_quad_indices.size()/2,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_flat->stopUseProgram();

	glDisable(GL_POLYGON_OFFSET_FILL);

	m_shader_color->startUseProgram();
	m_shader_color->sendViewMatrix(viewMatrix);
	m_shader_color->sendProjectionMatrix(projectionMatrix);
	glUniform3f(m_shader_color->idOfColorUniform, 0.0f,0.0f,0.0f);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo2);
	glDrawElements(GL_LINES, m_nb_ind_edges,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_color->stopUseProgram();
}

void MeshQuad::clear()
{
	m_points.clear();
	m_quad_indices.clear();
}

int MeshQuad::add_vertex(const Vec3& P)
{
    m_points.push_back(P);
    return m_points.size()-1;
}


void MeshQuad::add_quad(int i1, int i2, int i3, int i4)
{
    m_quad_indices.push_back(i1);
    m_quad_indices.push_back(i2);
    m_quad_indices.push_back(i3);
    m_quad_indices.push_back(i4);
}

void MeshQuad::convert_quads_to_tris(const std::vector<int>& quads, std::vector<int>& tris)
{
	tris.clear();
	tris.reserve(3*quads.size()/2); // 1 quad = 4 indices -> 2 tris = 6 indices d'ou ce calcul (attention division entiere)

    int i = 0;
    int size = quads.size();
    for(i=0 ; i < size; i+=4){
        tris.push_back(quads.at(i));
        tris.push_back(quads.at(i+1));
        tris.push_back(quads.at(i+3));

        tris.push_back(quads.at(i+1));
        tris.push_back(quads.at(i+2));
        tris.push_back(quads.at(i+3));



    }

	// Pour chaque quad on genere 2 triangles
	// Attention a respecter l'orientation des triangles
}



void MeshQuad::convert_quads_to_edges(const std::vector<int>& quads, std::vector<int>& edges)
{
	edges.clear();
	edges.reserve(quads.size()); // ( *2 /2 !)

    int i = 0;
    int j = 0;
    int sizeQuads = quads.size();

    auto doesExist = [&] (int a, int b) -> bool{
        for(j=0;j<(int)edges.size()-1;j+=2){
            if(quads.at(b)==edges.at(j) && quads.at(a) == edges.at(j+1)){
                return true;
            }
        }
        return false;
    };

    for(i=0;i<sizeQuads;i+=4){
        if(!doesExist(i,i+1)){
          edges.push_back(quads.at(i));
          edges.push_back(quads.at(i+1));
        }
        if(!doesExist(i+1,i+2)){
          edges.push_back(quads.at(i+1));
          edges.push_back(quads.at(i+2));
        }
        if(!doesExist(i+2,i+3)){
          edges.push_back(quads.at(i+2));
          edges.push_back(quads.at(i+3));
        }
        if(!doesExist(i+3,i)){
          edges.push_back(quads.at(i+3));
          edges.push_back(quads.at(i));
        }
    }


	// Pour chaque quad on genere 4 aretes, 1 arete = 2 indices.
	// Mais chaque arete est commune a 2 quads voisins !
	// Comment n'avoir qu'une seule fois chaque arete ?

}


void MeshQuad::create_cube()
{
	clear();
	// ajouter 8 sommets (-1 +1)
    int a = add_vertex(*new Vec3(0,0,0));
    int b = add_vertex(*new Vec3(1,0,0));
    int c = add_vertex(*new Vec3(1,1,0));
    int d = add_vertex(*new Vec3(0,1,0));
    int e = add_vertex(*new Vec3(0,0,1));
    int f = add_vertex(*new Vec3(1,0,1));
    int g = add_vertex(*new Vec3(1,1,1));
    int h = add_vertex(*new Vec3(0,1,1));

	// ajouter 6 faces (sens trigo)
    add_quad(d,c,b,a);
    add_quad(e,h,d,a);
    add_quad(h,e,f,g);
    add_quad(c,g,f,b);
    add_quad(b,f,e,a);
    add_quad(h,g,c,d);


	gl_update();
}

Vec3 MeshQuad::normal_of_quad(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{

    Vec3 v1 = cross(B-A,D-A);
    Vec3 v2 = cross(D-C,B-C);

    Vec3 res = Vec3((v1.x + v2.x)/2,(v1.y + v2.y)/2,(v1.z + v2.z)/2);

    qDebug() << res.x;

    return(normalize(res));

	// Attention a l'ordre des points !
	// le produit vectoriel n'est pas commutatif U ^ V = - V ^ U
	// ne pas oublier de normaliser le resultat.

}

float MeshQuad::area_of_quad(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{

    Vec3 v1 = cross(B-A,C-A);
    Vec3 v2 = cross(D-B,D-C);

    double R1 = length(v1)/2;
    double R2 = length(v2)/2;

    return((float)R1+R2);

	// aire du quad - aire tri + aire tri
	// aire du tri = 1/2 aire parallelogramme
	// aire parallelogramme: cf produit vectoriel


}


bool MeshQuad::is_points_in_quad(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
    Vec3 n = normal_of_quad(A,B,C,D);
    Vec3 v1 = cross(n,B-A);
    Vec3 v2 = cross(n,C-B);
    Vec3 v3 = cross(n,D-C);
    Vec3 v4 = cross(n,A-D);

    double p1 = dot(v1,P);
    double p2 = dot(v2,P);
    double p3 = dot(v3,P);
    double p4 = dot(v4,P);

    double da = dot(v1,A);
    double db = dot(v2,B);
    double dc = dot(v3,C);
    double dd = dot(v3,D);

    if(p1>da && p2 > db && p3 > dc && p4 > dd)
        return true;
    return false;


	// On sait que P est dans le plan du quad.

	// P est-il au dessus des 4 plans contenant chacun la normale au quad et une arete AB/BC/CD/DA ?
	// si oui il est dans le quad

}

bool MeshQuad::intersect_ray_quad(const Vec3& P, const Vec3& Dir, int q, Vec3& inter)
{
    // recuperation des indices de points
    int i1 = m_quad_indices.at(q*4);
    int i2 = m_quad_indices.at(q*4+1);
    int i3 = m_quad_indices.at(q*4+2);
    int i4 = m_quad_indices.at(q*4+3);

	// recuperation des points
    Vec3 p1 = m_points.at(i1);
    Vec3 p2 = m_points.at(i2);
    Vec3 p3 = m_points.at(i3);
    Vec3 p4 = m_points.at(i4);

	// calcul de l'equation du plan (N+d)
    Vec3 N = normal_of_quad(p1,p2,p3,p4);
    float d = dot(N,p1);


	// calcul de l'intersection rayon plan
	// I = P + alpha*Dir est dans le plan => calcul de alpha
    float alpha = (d-dot(N,P))/(dot(N,Dir));



	// alpha => calcul de I
     Vec3 I = P + (alpha*Dir);


	// I dans le quad ?
     if(is_points_in_quad(I, p1,p2,p3,p4)){
        inter = I;
        return true;
     }
     return false;
}


int MeshQuad::intersected_visible(const Vec3& P, const Vec3& Dir)
{
	// on parcours tous les quads
    int i = 0, res = -1;
    Vec3 inter;
    float dist = std::numeric_limits<float>::max();
    float d2;
    for(i=0;i<(int)m_quad_indices.size()/4;i++){
        // on teste si il y a intersection avec le rayon
        if(intersect_ray_quad(P,Dir,i, inter)){
            d2 = length(P-inter);
            // on garde le plus proche (de P)
            if(d2 < dist){
                dist = d2;
                res = i;
            }
        }
    }

    return res;
}


Mat4 MeshQuad::local_frame(int q)
{
	// Repere locale = Matrice de transfo avec
	// les trois premieres colones: X,Y,Z locaux
	// la derniere colonne l'origine du repere


	// ici Z = N et X = AB
	// Origine le centre de la face
	// longueur des axes : [AB]/2

	// recuperation des indices de points
    int i1 = m_quad_indices.at(q*4);
    int i2 = m_quad_indices.at(q*4+1);
    int i3 = m_quad_indices.at(q*4+2);
    int i4 = m_quad_indices.at(q*4+3);
	// recuperation des points
    Vec3 p1 = m_points.at(i1);
    Vec3 p2 = m_points.at(i2);
    Vec3 p3 = m_points.at(i3);
    Vec3 p4 = m_points.at(i4);

	// calcul de Z:N puis de X:arete on en deduit Y
    Vec3 N = normal_of_quad(p1,p2,p3,p4);
    Vec3 X = normalize(p2-p1);
    Vec3 Y = cross(N,X);

	// calcul du centre
    Vec3 ctr = p1+p2+p3+p4;
    ctr.x = ctr.x / 4;
    ctr.y = ctr.y / 4;
    ctr.z = ctr.z / 4;

	// calcul de la taille

    float taille = length(p2-p1)/2;

	// calcul de la matrice



    return (Mat4(Vec4(X,0),Vec4(Y,0),Vec4(N,0),Vec4(ctr,1)) * scale(taille,taille,taille));
}

void MeshQuad::extrude_quad(int q)
{
	// recuperation des indices de points
    int i1 = m_quad_indices.at(q*4);
    int i2 = m_quad_indices.at(q*4+1);
    int i3 = m_quad_indices.at(q*4+2);
    int i4 = m_quad_indices.at(q*4+3);
	// recuperation des points
    Vec3 p1 = m_points.at(i1);
    Vec3 p2 = m_points.at(i2);
    Vec3 p3 = m_points.at(i3);
    Vec3 p4 = m_points.at(i4);
	// calcul de la normale
    Vec3 N = normal_of_quad(p1,p2,p3,p4);


	// calcul de la hauteur

	// calcul et ajout des 4 nouveaux points

	// on remplace le quad initial par le quad du dessu

	// on ajoute les 4 quads des cotes

	gl_update();
}


void MeshQuad::decale_quad(int q, float d)
{
	// recuperation des indices de points

	// recuperation des (references de) points

	// calcul de la normale

	// modification des points

	gl_update();
}


void MeshQuad::shrink_quad(int q, float s)
{
	// recuperation des indices de points

	// recuperation des (references de) points

	// ici  pas besoin de passer par une matrice
	// calcul du centre

	 // modification des points

	gl_update();
}


void MeshQuad::tourne_quad(int q, float a)
{
	// recuperation des indices de points

	// recuperation des (references de) points

	// generation de la matrice de transfo:
	// tourne autour du Z de la local frame
	// indice utilisation de glm::inverse()

	// Application au 4 points du quad


	gl_update();
}

