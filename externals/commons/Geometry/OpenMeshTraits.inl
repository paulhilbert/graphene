template <class OpenMeshType>
typename OpenMeshTraits<OpenMeshType>::Size OpenMeshTraits<OpenMeshType>::numPoints(const MeshType& mesh) {
	return mesh.n_vertices();
}

template <class OpenMeshType>
std::vector<typename OpenMeshTraits<OpenMeshType>::VertexId> OpenMeshTraits<OpenMeshType>::vertexIdSet(const MeshType& mesh) {
	std::vector<VertexId> pointSet(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) pointSet[i++] = it.handle().idx();
	return pointSet;
}

template <class OpenMeshType>
std::vector<typename OpenMeshTraits<OpenMeshType>::PointType> OpenMeshTraits<OpenMeshType>::pointSet(const MeshType& mesh) {
	std::vector<PointType> pointSet(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) pointSet[i++] = mesh.point(it.handle());
	return pointSet;
}

template <class OpenMeshType>
void OpenMeshTraits<OpenMeshType>::mapPoints(MeshType& mesh, std::function<void (PointType&)> func) {
	for (auto vIt=mesh.vertices_begin(); vIt!=mesh.vertices_end(); ++vIt) {
		func(std::ref(mesh.point(vIt.handle())));
	}
}

template <class OpenMeshType>
typename OpenMeshTraits<OpenMeshType>::Size OpenMeshTraits<OpenMeshType>::numFaces(const MeshType& mesh) {
	return mesh.n_faces();
}

template <class OpenMeshType>
std::set<typename OpenMeshTraits<OpenMeshType>::FaceId> OpenMeshTraits<OpenMeshType>::faceIdSet(const MeshType& mesh) {
	std::set<FaceId> faces;
	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) faces.insert(it.handle().idx());
	return faces;
}

template <class OpenMeshType>
std::vector<typename OpenMeshTraits<OpenMeshType>::VertexId> OpenMeshTraits<OpenMeshType>::faceVertices(const MeshType& mesh, const FaceId& face) {
	auto faceHandle = mesh.face_handle(face);
	std::vector<VertexId> vertices;
	for (auto fvIt = mesh.cfv_iter(faceHandle); fvIt; ++fvIt) {
		vertices.push_back(static_cast<VertexId>(fvIt.handle().idx()));
	}
	return vertices;
}

template <class OpenMeshType>
std::vector<typename OpenMeshTraits<OpenMeshType>::PointType> OpenMeshTraits<OpenMeshType>::facePoints(const MeshType& mesh, const FaceId& face) {
	auto faceHandle = mesh.face_handle(face);
	std::vector<PointType> vertices;
	for (auto fvIt = mesh.cfv_iter(faceHandle); fvIt; ++fvIt) {
		vertices.push_back(mesh.point(fvIt.handle()));
	}
	return vertices;
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::Normal OpenMeshTraits<OpenMeshType>::faceNormal(const MeshType& mesh, const FaceId& face) {
	auto faceHandle = mesh.face_handle(face);
	auto normal = mesh.normal(faceHandle);
	Normal result(normal[0], normal[1], normal[2]);
	result.normalize();
	return result;
}

template <class OpenMeshType>
std::set<typename OpenMeshTraits<OpenMeshType>::FaceId> OpenMeshTraits<OpenMeshType>::adjacentFaces(const MeshType& mesh, FaceId f) {
	const auto& fHandle = mesh.face_handle(f);
	std::set<FaceId> adj;
	for (auto ffIt=mesh.cff_iter(fHandle); ffIt; ++ffIt) {
		adj.insert(ffIt.handle().idx());
	}
	return adj;
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::ScalarType OpenMeshTraits<OpenMeshType>::norm(const PointType& p) {
	return p.length();
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::PointType OpenMeshTraits<OpenMeshType>::crossP(const PointType& p0, const PointType& p1) {
	return p0 % p1;
}
