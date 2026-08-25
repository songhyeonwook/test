import open3d as o3d

# PLY 파일 읽기
ply = o3d.io.read_point_cloud("/home/unicon/hit0327.ply")

# PCD 파일로 저장
o3d.io.write_point_cloud("output.pcd", ply)

print("PLY -> PCD 변환 완료")
