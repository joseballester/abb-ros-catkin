
MODULE Module1
	PROC calibr()
		MoveAbsJ [[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, z50, tool0\WObj:=currentWobj;
	ENDPROC
	PROC corners()
		MoveJ [[657.44,-714.94,649.38],[1.35576E-05,-0.741001,-0.671504,-2.74422E-06],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, z50, tool0\WObj:=currentWobj;
		MoveL [[693.64,-743.59,3.51],[0.0391733,-0.970375,-0.123338,0.204021],[-1,-1,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, fine, toolTest1\WObj:=currentWobj;
		MoveL [[697.66,755.72,-6.80],[0.039154,-0.970378,-0.123341,0.204013],[0,0,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, fine, toolTest1\WObj:=currentWobj;
		MoveJ [[657.45,-714.93,649.37],[2.03763E-05,-0.740993,-0.671512,1.48382E-07],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, fine, tool0\WObj:=currentWobj;
		MoveL [[747.50,-717.85,74.70],[0.0391824,-0.970378,-0.123295,0.204033],[-1,-1,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, fine, tool0\WObj:=currentWobj;
		MoveL [[747.55,779.90,67.77],[0.0391406,-0.970378,-0.123319,0.204029],[0,0,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v1000, fine, tool0\WObj:=currentWobj;
	ENDPROC

ENDMODULE