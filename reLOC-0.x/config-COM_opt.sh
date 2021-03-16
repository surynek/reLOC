echo "/*====*/"
echo "//"
echo "//"
echo "//"
echo "// Compilation configuration file for "`cat product`" package - optimized variant."
echo "//"
echo "/*----*/"
echo ""
echo "#ifndef __COMPILE_H__"
echo "#define __COMPILE_H__"
echo ""
echo ""
echo "/*----*/"
echo ""
echo "namespace "`cat namespace`
echo "{"
echo ""
echo "//#define sDEBUG      1"
echo "#define sVERBOSE    1" 
echo "#define sSTATISTICS 1"
echo "#define sUSE_COMSPS 1"
echo ""
echo ""
echo "/*----*/"
echo ""
echo "} // namespace"`cat namespace`
echo ""
echo "#endif /* __COMPILE_H__ */"
