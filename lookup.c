const signed char lookuptable[4][4][4][4][4][8] __attribute__((section(".text"))) = {{{{{{81, 80, 66, -27, 7, 69, 24, 55, },{90, 90, 86, 82, 90, 90, 50, 90, },{46, 20, 46, 54, 90, 71, 64, 83, },{79, 58, 83, 63, 69, 60, 47, 46, },},{{90, 66, 36, 42, 85, 86, 36, 90, },{64, 70, 90, 64, 87, 90, 75, 35, },{90, 59, 50, 44, 90, 53, 90, 81, },{47, 66, 57, 90, 90, 24, 90, 56, },},{{47, 41, 71, 51, 63, 33, 90, 11, },{-8, 90, 30, 67, 41, 88, 16, 55, },{90, 90, 67, -23, 61, 82, 64, 89, },{79, 56, 78, 55, 40, 50, 40, 75, },},{{50, 11, 88, 76, 3, 35, 87, 86, },{36, 74, 47, 27, 90, -52, 75, 67, },{72, 90, 90, 32, 13, 60, 83, 86, },{85, 69, 90, 90, 74, 46, 50, 75, },},},{{{87, 57, 40, 90, 87, 16, 30, 83, },{63, 31, 34, 70, 90, 69, 74, 73, },{63, 39, 90, 68, 47, 72, 84, 63, },{49, 90, 62, 78, 90, 66, 50, 47, },},{{90, 90, 11, 36, 90, 51, 90, 74, },{55, 88, 27, 46, 90, 90, 90, 15, },{45, 67, 40, 53, -3, 90, 42, 57, },{90, 57, 44, 2, 45, 59, 53, 75, },},{{90, 90, 61, 59, 68, 39, 50, 38, },{50, 90, 90, 61, 37, 77, 60, 90, },{53, 87, 77, 48, 69, -8, 90, 53, },{90, 84, 78, 66, 70, 82, 77, 90, },},{{74, 82, 90, 90, 61, 21, 57, 88, },{20, 51, 24, 10, 45, 34, 78, 90, },{70, 90, 38, 12, 38, 76, 87, 67, },{-6, 65, 58, 73, 62, 88, 5, 90, },},},{{{48, -11, 55, 8, 62, 21, 80, 62, },{75, 86, 80, -4, 66, 18, 85, 87, },{-3, 17, 74, -2, 90, 67, 59, 90, },{59, -22, 71, 71, 55, 76, 26, 57, },},{{62, 75, 80, 67, 50, 65, 63, 88, },{73, 73, -21, 72, 44, 63, 63, 90, },{86, 44, 74, 84, 85, 88, 79, 71, },{74, 77, 85, 59, 90, 85, 48, 90, },},{{69, 90, 37, 78, 89, 58, -11, 58, },{90, 38, 90, 58, -28, 75, 7, 87, },{49, 90, 90, 45, 83, 75, -6, 62, },{86, 72, 61, 90, 90, 80, 90, 90, },},{{81, 77, 90, 90, 19, 26, 59, 90, },{90, 90, 84, 61, 70, 89, 69, 90, },{42, 71, 82, 69, 90, 90, 83, 80, },{90, 70, 90, 62, 74, 37, 20, 69, },},},{{{77, 90, 60, 54, 14, -7, 25, 90, },{69, 90, 23, 58, 43, -19, -25, 90, },{83, 18, 82, 90, 68, 28, 14, 33, },{61, 64, 75, 73, -89, 90, 89, 47, },},{{90, 61, 34, 50, 76, 90, 42, 83, },{-15, 90, 90, 64, 47, 78, 84, 62, },{46, 90, 90, 90, 76, 86, 90, 40, },{90, 90, 26, 41, 68, 90, 21, 90, },},{{71, 28, 90, 66, 90, 17, 90, 11, },{62, 78, 59, 90, 90, 74, 57, 74, },{71, 70, 78, 73, 51, 80, 14, -39, },{90, 45, 51, 83, 24, 72, 74, 64, },},{{90, 58, -6, 32, 59, 90, 77, 61, },{62, 90, 16, 43, 89, 90, 72, 68, },{47, -44, 38, 21, 72, 43, 90, 40, },{90, 81, 90, 64, 45, 48, 41, 58, },},},},{{{{72, 82, 71, 46, 87, 69, 37, 67, },{90, 19, 61, -2, 80, 90, 76, 16, },{-12, 90, 20, 90, 79, 30, 90, 44, },{34, 40, 87, 62, 48, 56, 90, 47, },},{{16, 6, 71, 87, 53, 82, 47, 77, },{55, 90, 78, 77, 73, 60, 80, -10, },{28, 65, 83, 73, 71, 36, 90, 84, },{50, 76, 90, 83, 39, 15, 73, 44, },},{{90, 59, 39, 40, 41, 69, 44, 54, },{90, 66, 46, 82, 48, 90, 45, 90, },{72, 89, 84, 88, 76, 29, 49, 45, },{73, 83, 72, 17, 87, 66, 78, 88, },},{{87, 85, 63, 17, 80, 28, 82, 54, },{85, 73, 67, 11, 90, 90, 2, 71, },{90, 72, 66, 67, 58, 48, 39, 85, },{-9, 81, 62, 14, 19, 57, 75, 76, },},},{{{90, 41, 82, 21, 69, 56, 47, 70, },{71, 86, 90, 41, 90, 61, 90, 80, },{14, 86, 13, 44, 73, 60, 75, 82, },{69, 90, 45, 46, 36, 61, 68, 78, },},{{89, 90, 71, 90, 90, 49, 90, 31, },{41, 90, 52, 42, 49, 37, 18, 17, },{62, 90, 19, 9, 90, 90, 21, 90, },{90, 75, 17, 90, 90, 90, 36, 67, },},{{90, 80, 82, 59, 88, 30, 61, 90, },{6, 50, -30, 53, 74, 52, 90, 45, },{33, 61, -22, 56, 63, 90, 18, 22, },{86, 44, 87, 6, 4, 32, 44, 90, },},{{62, 90, 52, 62, 67, 71, 86, 87, },{48, 90, 90, 90, 32, 46, 90, 9, },{90, 80, 53, 90, 44, 90, 90, 90, },{65, 70, 82, 58, 61, 44, 19, 40, },},},{{{76, 80, 41, 47, 73, 27, 90, 78, },{24, 69, 90, 64, 59, 55, 69, 76, },{64, 53, 66, 90, 90, 71, 65, 87, },{38, 90, 89, 90, 61, 90, 64, 66, },},{{90, 44, 77, 51, 90, 50, 88, 66, },{90, 35, 79, 69, 75, 71, 43, 62, },{75, 83, 90, 73, 61, 13, 67, 67, },{69, 76, 54, 50, -6, 67, 19, 90, },},{{38, 81, 82, 85, 90, 79, 18, 57, },{29, 90, 74, 11, 4, 46, 90, 90, },{79, 90, 90, 78, 33, 11, 90, 87, },{66, 80, 6, 39, -38, 61, 30, 90, },},{{65, 40, 88, 34, 85, 90, 90, 87, },{22, 90, 76, 54, 52, 37, 64, -28, },{90, 53, 36, 43, 60, 78, 58, 33, },{11, 41, 52, 90, 90, 54, 45, 63, },},},{{{-22, 9, 32, 90, 49, 62, 66, 80, },{77, 23, 59, 90, -12, -2, 49, 85, },{90, 90, 57, 55, 49, 69, 21, 77, },{74, 70, 90, 90, 36, 90, 21, 66, },},{{46, 66, 72, 90, 33, 78, -19, 65, },{71, 74, 90, -8, 54, 54, 63, 39, },{88, -22, 90, 62, 74, 74, 83, 71, },{83, 29, 72, 48, 50, 44, 90, 31, },},{{2, 67, 30, 90, 90, 11, 82, 90, },{23, 55, 77, -1, 52, 90, 63, 56, },{84, 59, 90, 45, 82, 68, -29, 85, },{89, 83, 78, 36, 44, 45, 30, 25, },},{{77, 72, 50, 37, 34, 75, 22, 57, },{54, 67, 87, 90, 7, 90, 78, 84, },{50, 80, 79, 51, 70, 66, 36, 87, },{34, 34, 50, 64, 60, 40, 50, 67, },},},},{{{{63, 80, 79, 90, -19, 90, 90, 86, },{66, 44, 69, 90, 69, 90, 88, 6, },{72, 66, 90, 84, 21, 6, 90, 55, },{8, 90, 41, 58, 71, 83, 90, 90, },},{{21, 26, 90, 61, 64, 57, 51, 90, },{90, 22, 64, -18, 68, 14, 90, 83, },{77, 68, 63, 70, 68, 64, 23, 72, },{73, 60, 14, 61, 36, -21, -21, 90, },},{{74, 32, 40, 85, 73, 87, 25, 59, },{90, 90, 77, 36, 50, 47, 89, 60, },{84, 19, 71, 69, 44, 51, 42, 75, },{54, 54, 90, 24, 70, 76, 47, 21, },},{{90, 44, 16, 56, 55, 90, 77, 49, },{40, 27, 63, 21, 72, 90, 60, 90, },{55, 63, 65, 20, 75, 78, 58, 67, },{90, 83, 78, 31, 90, 90, 82, 41, },},},{{{-32, 26, 38, 62, 37, 84, 57, 34, },{77, 72, 31, 52, 44, 90, 74, 75, },{76, 76, -16, 90, 46, 22, 58, 90, },{18, 28, 34, 55, -1, 40, 29, 38, },},{{88, 64, 42, 90, 89, 90, 17, 82, },{52, 59, 65, 89, 90, 88, 24, 75, },{-4, 83, 90, 37, 63, 90, 90, 80, },{61, 85, 90, 90, 31, 87, 57, 90, },},{{90, 45, 75, 73, 13, 72, 9, 86, },{90, 66, 85, -34, 59, 60, 90, 24, },{87, 54, 66, 43, 90, 18, 79, 72, },{64, 18, 56, 90, 67, 72, 50, 79, },},{{45, 57, 89, 75, 38, 74, 61, 24, },{70, 90, 49, 84, 62, 77, 66, 87, },{70, 55, 66, 27, 64, -28, 66, 83, },{71, 80, 29, 48, 76, 21, 43, 55, },},},{{{81, 63, 90, 61, 90, 44, 25, 75, },{90, -13, 90, 44, -2, 78, 57, 88, },{16, 49, -8, 75, 86, 62, 50, 84, },{56, 80, 38, 63, -6, 49, 80, 90, },},{{51, 62, 73, 51, 82, 52, 62, 79, },{39, 84, 76, 52, 90, 61, 14, 90, },{32, 76, 90, 66, 17, 83, 37, 74, },{81, 51, 43, 66, 90, 68, 90, 50, },},{{65, 90, 70, 26, 1, 67, 79, 73, },{33, 27, 84, 31, 90, 76, 32, 23, },{3, 53, 37, 72, 35, 50, 90, 65, },{69, 84, -1, 90, 72, 83, 90, 64, },},{{77, -28, 49, 21, 82, 83, 68, 79, },{67, 85, 77, 40, 90, 65, 90, 72, },{17, 67, 43, 41, 84, 72, -1, 76, },{73, -21, 43, 26, 49, 90, 76, 87, },},},{{{90, 78, 57, 11, 78, 86, 90, 55, },{90, 40, 70, 89, 90, 15, 90, 86, },{62, 73, 90, 46, 28, 78, 35, 90, },{36, 90, 56, 82, 86, 90, 79, 51, },},{{82, 64, 63, 52, 61, 64, 90, 34, },{84, 45, 47, 61, 66, 48, 57, 65, },{79, 47, 66, 48, -5, 79, 90, 63, },{38, 90, 34, 90, 54, 74, -23, 90, },},{{33, 90, 90, -28, 90, 54, 90, 31, },{74, 90, 90, 90, 54, 61, 74, 58, },{90, 49, 56, 63, 52, 55, 61, 88, },{90, 90, 65, 61, 59, 61, 69, 33, },},{{50, 72, 77, 90, 77, 90, 74, 81, },{64, 7, 84, 53, 43, 69, 37, 60, },{90, 58, 63, 38, 78, 51, 71, 90, },{65, 90, 32, 90, 68, 24, 24, 81, },},},},{{{{57, 90, 40, 32, 58, 49, 62, -10, },{60, 64, 9, 82, 7, 90, 74, 49, },{70, 56, -6, 4, 84, 90, 9, 90, },{-27, 90, -13, 52, 90, 90, 90, 73, },},{{32, 65, 41, 45, 72, 90, 36, 62, },{27, 35, 15, -6, 78, 75, 90, 29, },{83, 29, 16, 56, 89, 83, 90, 90, },{77, 23, 44, 82, 79, 41, 75, 50, },},{{64, 90, 90, 63, 81, 74, -9, 77, },{79, 9, 64, 89, 81, 78, 48, 19, },{44, 60, 84, 67, 90, 79, 90, 84, },{57, 66, 39, 89, 26, 84, 75, 65, },},{{87, 4, 90, 45, 87, 90, 90, 55, },{18, 64, 32, 83, 49, 90, 63, 75, },{51, 70, 44, 31, 79, 55, 90, 47, },{88, 28, 81, 44, 69, -20, 53, 56, },},},{{{79, -4, 43, 90, 29, 61, 90, 56, },{32, 87, 90, 90, 17, 68, 82, 83, },{90, 21, 43, 0, 90, 90, 90, 40, },{39, -11, 62, 39, -33, 57, 48, 72, },},{{26, 88, 82, 31, 68, 44, 72, 68, },{-5, 77, 71, 36, 90, 73, 29, 14, },{90, 16, 78, 42, 89, 85, 90, 73, },{56, 90, 85, 89, 90, 75, 43, 24, },},{{86, 66, 63, 0, 88, 18, 58, 37, },{32, 64, 79, 90, 25, 83, 90, -19, },{76, 21, 43, 79, -20, 90, 52, 78, },{65, 90, 90, 82, 90, 57, -3, 61, },},{{34, 53, 25, 88, 8, 66, 90, 90, },{12, 90, 22, 34, 70, 90, 31, 90, },{22, -13, 90, 90, 82, 82, 71, 68, },{66, 90, 63, 72, 81, 65, 90, 62, },},},{{{72, -27, 75, 72, 90, 90, 90, 60, },{-18, 65, 30, 50, 90, -81, 82, 49, },{76, 35, 88, 41, 64, 72, 90, 28, },{90, 62, 64, 90, 68, 90, 0, 85, },},{{52, 64, 15, 67, 90, 78, 32, 84, },{80, 36, 28, 90, 55, 74, 90, 81, },{9, 90, 90, 47, 85, 90, 81, 87, },{56, 64, 77, 90, 55, 90, 64, 38, },},{{41, 90, 46, 90, -7, 18, 90, 66, },{63, 10, 69, 50, 79, 86, 90, 61, },{84, 41, 85, 68, 73, -1, 2, 89, },{39, -20, 54, 59, 90, 54, 62, 90, },},{{-14, 45, 90, 75, 42, 90, 90, 74, },{19, 77, 53, 56, 83, 90, 26, 64, },{79, 17, 79, 88, -6, 90, 65, 63, },{68, 13, 90, 34, 74, 11, 87, 77, },},},{{{44, 35, 90, 66, 77, 58, 58, 76, },{87, 44, 18, 60, 44, 73, 62, 90, },{90, 71, 90, 80, 23, 57, 45, 46, },{90, 65, 5, 90, 21, -29, 66, 90, },},{{67, 56, 47, 90, 87, 74, 72, 71, },{29, 90, 47, 53, 90, 90, 39, 8, },{68, 64, 90, 39, 38, 90, 29, 90, },{72, 89, 84, 90, 81, 26, 49, 84, },},{{90, 90, -54, 68, 73, 19, 81, 90, },{-6, 90, 80, 61, 90, 20, 30, 71, },{0, 54, 73, 59, 90, 55, 55, 71, },{90, 19, 49, 45, 46, 90, -19, 60, },},{{78, 77, 6, 73, 70, 22, 59, 86, },{31, 77, 25, 70, 77, 59, 62, 54, },{90, 90, 64, 41, 88, 83, 84, 86, },{90, 90, 77, 84, 70, 73, 40, 90, },},},},},{{{{{23, 63, 13, 73, 82, 50, 13, 47, },{90, 90, 73, 62, 64, 26, 14, 48, },{19, 45, 85, 48, 90, 57, 52, 39, },{80, 28, -44, 51, 67, 81, 72, 62, },},{{33, 90, 31, 26, 40, 49, 44, 68, },{57, 73, 48, 84, 61, 81, 90, 77, },{90, 85, 90, 46, 17, 73, 58, 33, },{43, 40, 90, 29, 6, 52, -4, 41, },},{{55, 60, 83, 65, 90, 90, 61, -5, },{39, 21, -9, 57, -7, 70, 87, 86, },{56, 63, 68, 39, 87, 18, 79, 61, },{86, 27, 72, 90, 90, 71, 57, 90, },},{{78, 66, 78, 66, 85, 81, 40, 7, },{90, 74, 90, 11, 19, 39, 86, 70, },{43, 66, 50, 1, 13, 90, 56, 88, },{63, 65, -15, 52, -18, 47, 74, 43, },},},{{{80, 32, 79, 35, 50, 40, 88, 54, },{41, 75, 90, 73, 90, 65, 90, 40, },{22, 81, 71, 90, 64, 70, 70, -7, },{31, 11, 79, 88, 65, 90, 59, 14, },},{{51, -32, 52, 77, 59, 90, 90, -7, },{90, 90, 68, 72, 87, 90, 51, 8, },{90, 59, 47, 67, 22, 90, 17, 56, },{75, 90, 60, 55, 90, 76, 7, 36, },},{{79, 73, 90, 49, 73, -4, 69, 45, },{73, 71, 66, 67, 90, 63, -17, 54, },{90, 54, 90, 62, 35, 52, 73, 76, },{-55, 70, 37, 73, 66, 75, 46, 64, },},{{67, 90, 72, -45, 90, 74, 83, 71, },{57, 74, 84, 90, 63, 90, 90, 77, },{90, -63, 1, 72, -65, 62, 70, 13, },{46, 90, 25, 53, 90, 75, 89, 90, },},},{{{76, 90, 90, 32, 59, 60, 90, 62, },{3, 40, -12, 90, -59, 33, 90, 76, },{59, 44, 78, 90, 37, 85, 90, 41, },{42, 90, 21, 58, 35, 56, 23, 90, },},{{90, 74, 69, 90, 11, 68, 19, 63, },{61, -1, 90, 71, 60, 28, 41, 28, },{62, 90, 36, 62, 68, 90, 74, 90, },{90, 58, 41, -19, 61, 90, 58, 69, },},{{-17, 3, 24, 71, 78, 90, 64, 22, },{78, 56, 48, 90, 62, 29, 88, 68, },{54, 90, 90, 90, 86, 62, 89, 90, },{78, 60, 46, 72, 47, 48, 53, 90, },},{{-10, 17, 11, 10, 72, 33, 23, 75, },{74, 90, 90, 90, 56, 88, 42, 61, },{28, 79, 45, 31, 51, 58, 83, 58, },{90, 64, 87, 90, 39, 47, 55, 90, },},},{{{53, 90, 78, 46, 49, 85, 80, 36, },{30, 52, 90, 75, 69, 84, 4, 60, },{90, 15, 90, 45, 43, 54, 67, 90, },{90, 40, 44, 83, 90, 7, 90, 41, },},{{29, 35, 82, 90, 73, 7, 20, 90, },{64, 90, 90, -2, 61, 80, 76, 64, },{90, 42, -8, 43, 90, 79, 90, 30, },{21, 30, 85, 60, 74, 72, 85, 70, },},{{55, 21, 74, 29, 90, 80, 58, -63, },{83, 90, 51, 90, 31, 87, 79, 77, },{16, -6, 40, 90, 79, 51, 83, 33, },{72, 82, 90, 31, 57, 90, 9, 76, },},{{68, 89, 74, 90, 37, 36, 47, 90, },{72, 90, 85, 56, 77, 60, 28, 31, },{62, 90, 50, 29, 76, 90, 71, 63, },{58, 75, 71, 90, 44, 90, 43, 40, },},},},{{{{71, -48, 73, 80, 6, 54, 68, 43, },{70, 37, -1, 90, 90, 46, 56, 65, },{32, 44, -3, 90, 90, -14, 43, 87, },{43, 90, 88, 50, -23, 19, 33, 65, },},{{40, 89, 2, 71, 90, 19, 82, 31, },{25, 90, 36, 90, 45, 34, 57, -20, },{44, 54, 90, -58, 52, 48, 90, 29, },{40, 73, 33, 52, 72, 90, 85, 70, },},{{85, 90, 67, 77, 85, 63, 60, 82, },{79, 90, 78, 19, 71, 90, 90, -7, },{50, 52, 90, 11, 90, 71, -13, 90, },{40, 90, 47, 90, 90, 63, 6, 11, },},{{17, 57, 90, 62, 32, 75, -9, 90, },{-24, 88, 90, 90, 90, 61, 49, 54, },{42, 67, 55, 61, 90, 19, 17, 90, },{90, 68, 33, 1, 83, 64, 20, 51, },},},{{{58, 80, 90, 69, 90, 49, 90, 32, },{90, 90, 41, -23, -3, 72, 50, 90, },{90, -6, 67, 80, 90, 66, 32, 58, },{90, 90, 90, 78, 63, 24, 65, 65, },},{{88, 61, 45, 76, 26, 89, 56, 63, },{48, 73, 42, 68, 90, 90, 65, 34, },{54, 80, -9, 78, 45, 51, 41, 74, },{75, 63, 90, 90, 47, 90, 90, 4, },},{{71, 90, 55, 64, 90, 90, 90, 52, },{14, 10, 75, -33, 90, 43, 41, 58, },{90, 90, 77, 64, 76, 7, 77, 87, },{90, 72, 90, 26, 38, 47, 75, 88, },},{{27, 90, 90, 90, 90, 80, 14, 48, },{18, 72, 90, 30, 55, 18, 71, 49, },{81, 63, 42, 42, 20, 90, 73, 89, },{90, -15, 48, 90, 79, 38, 79, 90, },},},{{{85, 37, 56, 54, 71, 72, 58, 77, },{69, 61, 85, 34, 66, 75, 75, 90, },{11, 65, -30, 29, -14, 80, 82, 54, },{63, -18, 24, 32, 69, 70, 75, -31, },},{{12, 90, 65, 60, 90, 32, 59, 52, },{90, 68, 63, 89, 60, 90, 65, 52, },{12, 75, -1, 61, 22, 85, 67, 39, },{73, -7, 13, 49, 33, 77, 33, 90, },},{{89, 87, 90, 62, 90, 55, 46, 64, },{73, 50, 68, 90, 42, 67, 90, 90, },{57, 1, 66, 42, 75, 80, 2, 88, },{15, 74, 78, 78, 90, 37, 38, 73, },},{{50, 46, 68, 31, 61, 90, 70, 90, },{54, 45, 90, 87, 18, 76, 86, 55, },{55, 67, 90, 46, 74, 90, 75, 70, },{79, 63, 90, 70, 82, 73, 21, 5, },},},{{{54, 82, 87, 90, 18, 87, 85, 13, },{87, 90, 43, 33, 42, 50, 34, 90, },{66, 68, 84, 72, 52, 67, 90, -5, },{90, 4, -39, 44, 37, 25, 9, 81, },},{{90, 62, 47, 65, 79, 44, 90, 90, },{69, 88, 17, 72, 90, 82, 90, 90, },{68, 75, -27, 77, 33, 84, 85, 27, },{61, 31, 62, 55, 66, 75, 88, 90, },},{{90, 40, 38, 62, 90, 52, 46, 89, },{49, 60, 1, 90, 37, 45, 75, 56, },{90, 41, 84, 90, 90, 82, 81, 90, },{6, 90, 79, -8, 82, 40, 90, 65, },},{{61, 33, 63, 81, 67, 53, 72, 74, },{83, 32, 46, -12, 54, -13, 53, 65, },{64, 33, 90, 12, 90, 90, 77, 16, },{36, 14, 79, 90, 5, 85, 80, 72, },},},},{{{{90, 40, -9, 48, 64, 70, 90, 28, },{59, 26, 78, 90, 90, 58, -2, 19, },{60, 51, 34, 63, 65, 83, 90, 90, },{83, 90, 64, 60, 52, 90, 90, 70, },},{{65, 58, 31, 51, 74, 80, 1, 34, },{90, 90, 21, 44, 90, 83, 90, 59, },{78, 33, 29, 63, 90, 90, 63, 18, },{43, 63, 90, 90, 88, -33, 88, 86, },},{{27, 90, 90, 19, 82, 69, 52, 37, },{88, 90, 87, 65, 45, 72, 50, 45, },{90, 67, 32, 90, 16, 34, 90, 90, },{27, 77, 90, 77, 72, 70, 59, 54, },},{{71, 89, 80, 67, 4, 81, 68, 90, },{37, 51, 75, 82, 90, 52, 48, 53, },{69, 90, 76, 69, 89, 53, 58, 58, },{82, 76, 23, 83, 69, 83, 87, 90, },},},{{{84, 65, 90, 87, 71, 65, 33, 24, },{18, 38, 70, 90, 22, 65, 90, 90, },{-5, 38, 66, 43, 87, 90, 88, 38, },{77, 51, 54, 90, 76, -6, 52, 60, },},{{90, 78, 90, 90, -26, 66, 56, 54, },{29, 84, 70, 32, 37, 90, 65, 90, },{18, 55, 48, 45, 47, 41, 90, 46, },{33, 23, 71, 78, 56, 11, 57, 41, },},{{43, 90, -14, 30, 88, 90, -30, 65, },{47, 74, 90, 62, 79, -44, 72, 68, },{63, 76, 30, 57, 37, 90, 90, 90, },{39, 3, 32, 59, 70, 90, -15, 48, },},{{49, 65, -14, 80, 90, 67, 66, 90, },{66, 6, 90, 51, 90, 90, 4, -9, },{71, 41, 71, 33, 76, -25, 90, 47, },{68, 69, 80, -8, 69, -42, 87, 43, },},},{{{76, 24, 48, 53, 53, 57, 44, 67, },{40, -7, 56, 32, 51, 65, 70, 10, },{11, 90, 1, 90, 90, 90, 44, 90, },{83, 73, 59, 37, 62, 69, 5, 50, },},{{55, 16, 73, 64, 90, 90, 23, 90, },{80, 73, 2, 29, 42, 57, 72, 70, },{73, 71, 60, 78, 90, 56, 42, 31, },{63, -3, -3, 90, 63, -3, 24, 44, },},{{69, 47, 57, 21, 49, 83, 63, 90, },{46, 47, 56, 53, 43, 90, 90, 19, },{90, 56, 29, 57, 90, 39, 48, 60, },{23, 22, 44, 60, 65, 62, 82, 90, },},{{36, 61, 90, -13, 90, 84, -4, 77, },{55, 25, 80, 54, 90, 37, 90, 85, },{42, 90, 27, 88, 58, 61, 43, 12, },{26, 85, 75, 90, -22, -1, 26, 90, },},},{{{90, 78, 53, 39, 64, 90, 71, 65, },{83, 35, 58, 49, 83, 63, 90, 17, },{82, 67, 90, 36, 90, 48, 3, 11, },{67, 72, 61, 90, 74, 90, 90, 82, },},{{47, 76, 90, 47, 71, 52, 68, 40, },{90, 62, 40, 65, 67, 90, 42, 90, },{90, 35, 74, 89, 90, 32, 31, 25, },{82, 63, 80, 75, 71, 62, 48, 12, },},{{90, 51, 90, 26, 79, 58, 60, 86, },{-6, 90, 86, 90, 77, 72, 24, 90, },{90, 16, 87, 90, 90, 47, 64, 90, },{28, 79, 81, 88, 31, 41, 87, 66, },},{{80, 68, 42, 82, -13, 26, 90, 85, },{63, 20, 43, 81, 90, 90, 34, 40, },{45, -5, 71, 74, 90, -33, 42, 47, },{83, 61, 76, 53, 90, 63, 54, 64, },},},},{{{{90, 69, 37, 34, 29, 80, 90, 75, },{7, 88, 85, -3, 84, 28, 90, 48, },{90, 31, 90, 77, 47, 49, 85, 86, },{46, 59, 68, 52, 80, 36, 61, 61, },},{{46, 89, 63, 26, 90, 90, 68, 82, },{90, 88, 90, 65, 89, 85, 18, 66, },{90, 67, 90, 37, 90, 90, 90, -10, },{63, 34, 33, 50, 19, 50, 82, 71, },},{{47, 83, 55, 44, 65, 79, 37, 15, },{3, 32, 78, 69, 4, 90, 65, 90, },{30, 74, 75, 65, 90, 41, 90, 90, },{57, 77, 35, 82, 53, 43, 41, 68, },},{{55, 81, 24, 62, 90, 52, 30, 31, },{74, 90, 59, 54, 90, 81, 29, 81, },{74, 66, 78, 62, 28, 8, 90, 90, },{90, 90, 69, 88, 90, 46, 84, 35, },},},{{{13, 73, 48, 49, 30, 49, 37, 11, },{80, 68, 90, 90, 62, 90, 50, 19, },{23, 25, 90, 43, 80, 65, 90, 22, },{32, 82, 87, 90, 89, 56, 18, 89, },},{{62, 53, 22, 49, 90, 40, 50, 34, },{27, 50, 66, 86, 73, 90, 16, 72, },{90, 68, 69, 90, 67, 61, 69, 79, },{12, 90, 90, 74, 90, 27, 55, 59, },},{{90, -8, 86, 67, 56, 15, 31, 40, },{27, 90, 54, 22, 42, 42, 51, 90, },{58, 53, 8, 90, 68, 72, 90, 90, },{82, 90, 25, 82, -34, 36, 90, 44, },},{{-60, 63, 60, 44, 81, 65, 61, 55, },{42, 77, 71, 67, 88, 90, 80, 52, },{57, 63, 90, 67, 68, 90, 11, 90, },{64, 90, 50, 65, 20, -9, 53, 90, },},},{{{37, 55, 50, 90, 90, 90, 90, 31, },{71, 90, 34, 81, 44, 45, 77, 80, },{90, 10, 90, -15, 80, 73, 48, 77, },{90, 15, 22, 87, 2, 83, 49, 90, },},{{71, 29, 87, 37, 90, 67, 39, 41, },{83, 62, 68, 51, 9, 14, 14, 57, },{90, 90, 57, 70, 90, 30, 60, 90, },{41, 90, 68, 50, 90, 6, -62, 25, },},{{88, 49, 90, 56, 90, 30, 62, 13, },{27, 56, 66, 80, 40, 58, 90, 30, },{5, 60, 83, 71, 39, -24, 72, 18, },{57, 58, 77, 60, 89, 61, -20, 6, },},{{51, 81, 30, 90, 90, 60, 49, 80, },{65, 90, 73, 88, 90, -10, 52, -68, },{20, 29, 56, 7, 46, 69, -11, 83, },{90, 90, 40, 28, 87, 86, 6, 86, },},},{{{65, 90, 51, 90, 47, 75, 86, 43, },{85, 83, 90, 73, 90, 68, 90, 90, },{23, 62, 90, 90, 90, -26, 27, 90, },{48, 73, 90, 66, 59, 68, 65, 90, },},{{80, 41, 69, 90, 58, 90, 21, 29, },{89, 90, 22, 63, 44, 22, 65, 67, },{43, 50, 36, 90, 90, 51, 1, 23, },{50, 42, 59, 90, 90, 70, -53, 28, },},{{76, 62, 57, 90, 90, 74, 75, 78, },{90, 32, 63, 59, 90, 51, 88, 71, },{77, 55, 80, 63, 78, 90, 85, 90, },{59, 52, 54, 48, 71, 40, 23, 53, },},{{3, 56, 63, 76, 90, 2, 82, 71, },{90, 90, 90, 13, 27, 57, 0, 90, },{62, 81, 46, 90, 90, 64, -15, 79, },{75, 15, 54, 90, 64, 33, 83, 90, },},},},},{{{{{87, 90, 69, 63, 66, 64, 49, -16, },{90, 76, 65, 63, 52, 38, 44, 70, },{63, 68, 72, 78, 84, 3, 13, 57, },{90, 56, -29, 90, 11, 20, 78, 90, },},{{79, 41, 13, 90, 90, 71, 56, 74, },{86, 90, 12, 80, 55, 77, 90, 26, },{68, 18, 52, 56, 83, 79, 90, 63, },{90, 84, 48, 72, 43, 66, 63, 90, },},{{74, 90, 84, 90, 65, 26, 90, 71, },{31, 33, 74, 90, 36, 59, 44, 77, },{90, 67, 52, -8, 67, 48, 58, 90, },{90, 67, 70, 90, 72, 18, 61, 19, },},{{74, -33, 68, 49, 90, 29, 90, 58, },{78, 54, 77, 90, 90, 65, 59, 90, },{52, 90, 51, 88, 40, 51, 60, 89, },{53, 36, -15, 81, 81, 13, 90, 90, },},},{{{35, 21, 58, 47, 74, 66, 90, 57, },{83, 90, 56, 5, 85, 90, -1, 57, },{71, 90, 49, 16, 68, 47, 17, 52, },{18, -14, 90, 15, 63, 77, -34, 36, },},{{90, 90, 5, 23, 39, 90, 47, 90, },{86, 30, 18, 90, 38, 90, 90, 8, },{90, 33, 18, 20, 73, 71, 90, 68, },{61, 90, 72, 15, 9, 49, 90, 53, },},{{76, 86, 71, 90, 67, -13, 58, 62, },{35, 47, 67, 41, 90, 90, 10, 12, },{79, 75, 31, 75, -47, 88, 43, 35, },{34, 72, 60, 45, -9, 59, 78, 8, },},{{-9, 23, 15, 40, 90, 45, 41, 67, },{87, 84, 90, 90, 48, 90, 38, 57, },{47, 90, 78, 64, 85, 74, 83, 90, },{28, 41, 24, 90, 30, 43, -14, 57, },},},{{{26, 25, 43, 60, 90, 73, 12, 82, },{90, 70, 25, 69, 43, 58, 81, 90, },{90, 85, 76, 90, 85, 22, 85, 64, },{80, 83, 47, 13, 90, 69, 66, 61, },},{{17, 45, 37, 90, 90, 74, 29, 52, },{40, 65, 85, 67, 80, 55, 68, 37, },{90, 90, 4, 26, 28, -11, 60, 51, },{77, 40, 33, 51, 73, 90, 43, 59, },},{{43, 70, 75, 46, 27, -6, 67, 53, },{54, 53, 44, 90, 69, 90, 69, 55, },{42, 82, 59, -3, 59, 80, -11, 90, },{81, -12, 1, 90, 80, 73, 41, 68, },},{{41, 83, 51, 63, 70, 90, 81, 65, },{50, 46, 70, 56, 60, 73, 62, 71, },{60, 0, 62, 67, 37, 23, 47, 48, },{89, 88, 85, 54, 54, 29, 77, 60, },},},{{{45, 18, 66, 25, 90, 55, 24, 50, },{29, 65, 75, 90, 90, 90, 30, 90, },{75, 47, 62, 90, 56, 90, 72, 51, },{67, 70, 90, 83, 45, 83, 90, 48, },},{{37, 90, 28, 66, 48, 59, 60, 44, },{74, 82, 86, -4, 24, 75, 50, 87, },{47, 90, 68, 82, 32, 68, 69, 57, },{62, 90, 90, 90, 36, 16, 72, 52, },},{{74, 90, 51, 59, 60, 42, 59, 21, },{24, 90, 70, 90, 44, 83, 88, 71, },{69, 75, 37, 64, 85, 34, 41, 41, },{90, 83, 17, 36, 25, -1, 41, 90, },},{{51, 61, 47, 86, 39, 90, 61, 42, },{66, 33, 85, 60, 90, 60, 35, 70, },{63, 90, 60, 46, 18, 55, 90, 60, },{90, 86, 90, 56, 90, 90, 90, 90, },},},},{{{{50, 77, 90, 35, 1, 90, 48, 38, },{90, 76, 65, 90, 90, 35, 84, 74, },{90, 90, 90, 20, 52, 59, 5, 90, },{4, 28, 65, 90, 80, 60, 70, 84, },},{{78, 58, 45, 90, 22, 28, 22, 61, },{69, 71, 90, 82, 73, 39, 42, 90, },{-17, 90, 79, 47, 60, 85, 8, 28, },{13, 1, 50, 25, 62, 37, 76, 63, },},{{85, 80, 90, 83, 87, 73, 55, 29, },{52, 76, 90, 90, 90, 48, 39, -25, },{66, 28, 77, 74, 90, 55, 90, 82, },{74, 90, 82, 38, 20, 73, 9, 78, },},{{88, 32, 72, 75, 68, 39, 43, 77, },{90, 24, 60, 84, 90, 90, 58, 53, },{90, 90, 90, 67, 87, 90, 24, 64, },{47, 90, 90, 54, 90, 90, 44, 82, },},},{{{57, 24, 76, 90, 25, 90, 13, 65, },{-2, 51, 41, 46, 41, 82, 90, 64, },{85, 67, 51, 54, 59, 30, 23, 90, },{52, 46, 61, 90, 72, 90, 62, 90, },},{{10, 55, 71, 70, 76, 67, 23, 69, },{82, 83, 90, 62, 59, 90, 51, 90, },{68, 70, 58, 79, 49, 46, 64, 66, },{90, 36, 61, 50, 90, 66, 11, 53, },},{{83, 56, -20, 64, 33, 90, 65, 90, },{88, 41, 90, 6, 66, 28, 53, 14, },{63, 69, 90, 90, 90, 90, 66, 37, },{90, 32, 49, 7, 90, 26, 68, 39, },},{{31, 22, 21, 53, 22, 39, 52, 90, },{43, 90, 29, 41, 49, 90, 54, 72, },{90, 77, 90, 28, 77, 52, 88, 57, },{47, 90, -41, 21, 4, -6, 59, 25, },},},{{{87, 69, 72, 60, 90, 33, 26, 63, },{-12, 75, 58, 56, 72, 68, 58, 36, },{-15, 26, 90, 11, 66, 90, 90, 89, },{43, 61, 19, 58, 68, 36, -44, 83, },},{{70, 29, 54, 17, 45, 90, 90, 88, },{30, 90, 71, 90, 54, 90, -8, -9, },{21, 12, 84, 21, 62, 86, 69, 85, },{8, 90, 68, 13, 59, 63, 90, 37, },},{{77, 44, 65, 40, 66, 65, 76, 74, },{90, 78, 77, 69, 59, 58, 76, 23, },{44, 84, 46, 81, 63, -3, 57, 85, },{44, 46, 32, 50, 67, 78, 60, 34, },},{{59, 66, 61, 88, 90, 89, 48, 56, },{90, 28, 90, 68, 62, 84, 90, 62, },{42, 63, 79, 40, 21, 68, 34, 15, },{83, 89, 68, 90, 47, 42, 90, 68, },},},{{{29, 90, 61, 63, 90, 79, 78, 52, },{79, 27, 60, 58, 34, 56, 50, 66, },{90, 81, 5, 90, 56, -20, 45, 66, },{90, 90, 45, -13, 90, 46, 90, 64, },},{{28, 81, 15, 16, 64, 29, 60, 86, },{90, 90, 90, 83, 65, 90, 50, 25, },{90, 73, 32, 62, -8, 71, 90, 28, },{-38, -5, 24, 85, 49, 54, 21, 90, },},{{90, 82, 81, 87, 64, 63, 48, 81, },{45, 68, 90, 90, 29, 63, 90, 77, },{31, 90, 61, 61, 69, 78, 89, 62, },{90, 64, 70, 56, 47, 50, 90, 68, },},{{31, 80, 90, 90, 67, 90, 21, 90, },{79, -32, 85, 90, 25, 51, 83, 7, },{14, 26, 69, 14, -42, 62, 40, 18, },{90, 89, 90, 60, 87, 59, 48, 90, },},},},{{{{90, 70, 90, 76, 81, 30, 72, 31, },{27, -5, 90, 58, 45, 29, 78, 89, },{64, 28, 90, 90, 71, 73, 36, 65, },{89, 90, 21, 62, 22, 76, 24, 83, },},{{45, 90, 90, 90, 88, 80, 90, 71, },{90, 90, 40, 52, 56, 33, 71, 41, },{50, 72, 3, 89, 90, 67, 80, -16, },{60, 0, 90, 84, 90, 82, 73, 67, },},{{80, 90, 90, 90, 25, 20, 90, 90, },{82, 90, 56, -5, 56, -39, 61, -21, },{83, 90, 89, 82, 25, 55, -12, 56, },{77, 49, 90, 57, 76, 55, 90, 70, },},{{36, 90, 69, -20, 90, 46, 17, 24, },{86, 54, 69, 87, 21, 78, 72, 84, },{49, 67, 46, 80, 35, 45, 90, 27, },{52, 20, 90, 66, 63, 56, 45, 51, },},},{{{49, 43, 38, 90, 79, 45, 79, 75, },{90, 90, 57, 70, -6, 47, 26, 66, },{19, 23, 16, 90, 56, 19, -8, 86, },{80, -31, 70, 65, 43, 14, -39, 7, },},{{74, 80, 51, 70, 90, 77, 85, 90, },{51, 73, 90, 90, 90, 62, 90, 83, },{89, 76, 90, 15, 31, 90, 57, 29, },{90, 90, 70, 90, 58, 44, 90, 18, },},{{78, 62, 19, 90, 27, 56, 26, 22, },{65, 12, 40, -2, 28, 90, 90, 59, },{90, 7, 87, 47, 90, 22, 59, 70, },{90, 36, 46, 48, 82, 72, 28, 61, },},{{61, 72, 90, 77, 60, 81, 90, 84, },{83, 60, 34, 75, 63, 87, 40, 90, },{90, 55, 90, 68, 72, 38, 87, 90, },{78, 72, 90, 50, 90, 83, 15, 52, },},},{{{66, 52, 52, 5, 48, 90, 69, 88, },{77, 64, 57, -10, 90, 46, 1, 90, },{90, 72, 74, 79, 54, 18, 77, 83, },{56, -2, 90, 90, 60, 90, 42, 84, },},{{77, 46, 8, -1, 67, 55, 52, 38, },{90, 74, 74, 51, 90, 39, 47, 10, },{83, 66, 85, 90, 63, 57, 67, 78, },{66, 48, 3, 90, 90, 85, 90, 42, },},{{61, 70, 41, 10, 73, 40, 39, 41, },{68, 76, 23, 90, 13, 47, 17, 69, },{22, 60, 5, 89, 90, 24, 74, 72, },{90, 90, 33, 90, -13, 81, 73, -6, },},{{27, 90, 29, 42, 82, 45, 35, 90, },{74, 90, 90, 41, 85, 65, 81, 29, },{77, 1, 61, 39, 43, 65, 72, 51, },{-16, 25, 87, 46, 54, 77, 39, 51, },},},{{{90, 65, 14, 40, 90, 64, 88, 70, },{33, 90, 21, 90, 60, 50, 76, 14, },{79, 24, 68, 51, 63, 67, 68, 79, },{90, 79, 57, 43, 30, 70, 50, -23, },},{{41, 53, 79, 89, 53, 75, 71, 75, },{24, 79, 90, 90, 37, 90, 62, 90, },{31, 71, 81, 54, 90, -3, 5, 78, },{90, 90, 90, 50, 90, 22, 16, 74, },},{{63, 82, 17, 68, 20, 46, 87, 90, },{90, 70, 29, 37, -25, 90, 48, 4, },{46, 30, 90, 68, 24, 90, 88, 78, },{78, 37, 63, 22, 90, 90, -5, 27, },},{{75, 48, 90, 86, 70, 67, 69, 56, },{50, 1, 21, 41, 90, 90, 67, 84, },{18, 61, 51, 37, 90, 41, 90, 90, },{87, 25, 31, 68, 59, 26, 40, 78, },},},},{{{{43, 66, 90, 39, 90, 66, 74, 90, },{34, 55, 90, 75, 57, 9, 90, 48, },{43, 31, 60, 90, 66, 33, 42, 90, },{64, 72, 14, 16, 30, 65, 60, 24, },},{{52, 90, 12, 46, 10, 62, 67, 90, },{59, 67, 40, 48, 12, 90, 42, 90, },{23, 32, 90, 81, 90, 88, 10, 84, },{53, 67, 57, 90, 90, -7, -17, -8, },},{{22, 88, 64, 35, 59, 65, 35, 90, },{38, 34, 72, 21, 69, -15, 33, 38, },{51, 31, 29, 54, 58, 60, 83, 90, },{81, 53, 21, 64, 50, 90, 70, 37, },},{{62, 51, 51, 90, 74, 66, 77, 64, },{58, -1, 50, 90, 56, 38, 56, 43, },{66, 90, 80, 90, 71, 11, 90, 90, },{67, 79, 83, 52, 90, 38, 51, 55, },},},{{{55, 41, 82, 65, 32, 70, 81, 55, },{72, 83, 86, 76, 57, -21, 71, 87, },{66, 83, 5, 90, 68, 16, 82, 37, },{73, 90, 90, 24, 13, 90, 12, 47, },},{{90, 90, 84, 68, -16, 75, 35, 74, },{75, 79, 35, 1, 54, 90, 82, 39, },{66, 46, 64, 1, 16, 33, 23, 90, },{-14, 59, 68, 39, 90, 90, 82, 69, },},{{90, 84, 90, 3, 90, 68, 86, 90, },{-6, 71, 70, -15, 49, 38, 49, 22, },{85, 90, 90, 90, 57, 69, -68, 46, },{43, 80, 67, 69, 47, 67, 32, 34, },},{{77, -5, 11, 90, 90, 57, 72, 21, },{90, 70, 78, 56, 51, 58, 46, 39, },{51, 80, 78, 73, 90, 90, 35, 88, },{90, 90, 90, 53, 83, 24, 90, 72, },},},{{{30, 36, 78, 61, 17, 90, 82, 3, },{13, 88, 70, 90, 90, 16, 19, 49, },{90, 30, 90, 90, 52, 79, 53, 77, },{67, 90, 90, 57, 20, 66, 66, 90, },},{{90, 42, 90, 62, 51, 78, 60, 79, },{52, -20, 83, 38, 75, 64, 57, 81, },{54, 38, 90, 70, 50, 38, 90, 90, },{67, 90, 60, 41, 44, 33, 85, -23, },},{{84, -9, 69, -10, 39, 40, 22, 82, },{90, 85, 41, 82, 71, 90, 23, 36, },{48, 35, 55, 35, 76, 24, 88, 79, },{40, 17, 73, 49, 90, 73, -27, 43, },},{{90, 79, 76, 66, 25, -34, 66, 80, },{62, 54, 69, 69, 38, 90, 16, 83, },{67, 65, 90, 90, 83, 47, 51, 90, },{90, 65, -74, 72, 48, 90, 88, 90, },},},{{{38, -19, 63, 42, 86, 28, -2, 37, },{90, 86, 90, 40, 87, 24, 90, 90, },{90, 64, 18, 53, 19, 41, 20, 89, },{90, 90, 90, 90, 5, 60, 47, 51, },},{{48, 82, 28, 42, 59, 55, 67, 73, },{42, 67, 86, 90, 40, 51, 50, 90, },{88, 61, 84, 90, 67, 75, -84, 17, },{53, 1, 90, -2, 35, 60, 57, -11, },},{{61, 3, 64, 90, 66, 72, 71, 58, },{90, 74, -52, 25, 75, 90, 63, 77, },{46, 77, 80, 90, 66, 70, 90, 43, },{73, 31, 86, 90, 49, 4, 80, 89, },},{{-39, 80, 61, 36, 81, 69, 9, 90, },{83, 90, 82, 62, 34, -32, 61, 41, },{90, 85, 84, 13, 44, 90, -6, -47, },{18, 90, 71, 90, 32, 73, 90, 53, },},},},},{{{{{8, 50, 57, 79, 49, 37, 90, 74, },{26, 20, 69, 30, 90, 52, 86, 90, },{69, 9, 25, 66, 68, 29, 20, 57, },{90, -64, -29, 90, 24, 69, 54, 90, },},{{90, 90, 49, 35, 90, 15, 84, 88, },{57, 77, 44, 90, -11, 33, 80, 38, },{90, 73, 77, 61, 88, 47, 23, 56, },{90, 36, 43, 36, 20, 62, 61, 62, },},{{86, 71, 17, 90, 71, 73, 90, 90, },{50, 1, 90, 11, 90, 90, 90, 79, },{30, 53, 60, 83, 0, 88, 72, 78, },{61, 56, 48, 77, 69, 41, 71, 71, },},{{90, 14, 90, 61, 11, 65, 73, 42, },{55, 77, 2, 50, 57, 68, 81, -5, },{79, 66, 39, 56, 34, 90, 45, 81, },{63, 90, 57, 90, 30, 72, 3, 5, },},},{{{73, 76, 60, 50, 27, 54, 52, 77, },{79, 15, 90, 90, 63, -6, 78, 3, },{79, 81, 30, 78, 77, 90, 83, 33, },{3, 84, -16, 90, 79, 13, 13, 90, },},{{69, 63, 90, 48, 64, 33, 75, 90, },{-36, 90, 71, 87, 53, 78, 60, 66, },{9, 71, 34, 16, 64, 14, 52, 90, },{90, 80, 19, 40, 49, 90, 48, 34, },},{{8, 72, -13, 50, 28, 34, 90, 68, },{59, 66, -2, 73, 86, 89, 48, 49, },{66, 83, 36, -13, -60, 90, 54, 90, },{62, 64, 72, 83, 71, 47, 90, 90, },},{{15, 29, 12, 62, 38, 80, 62, 90, },{80, 90, 80, 90, 90, 90, 77, 20, },{84, 79, 68, 54, 87, 36, 68, 83, },{78, 40, 81, 32, 90, 90, 36, 55, },},},{{{89, 77, 87, 64, 74, 65, 87, 90, },{-18, 46, 56, 55, 71, 22, 90, 86, },{61, 90, 70, 35, 70, 90, 83, 88, },{90, 66, 23, 40, 67, 80, 38, 24, },},{{77, 86, 11, 54, 72, 28, 37, 4, },{90, 89, 20, 48, 1, 37, 90, 22, },{-3, 75, 80, 16, 49, 74, 78, 61, },{52, 84, 28, 39, 37, 15, 46, 46, },},{{46, 55, 90, 90, 66, 90, 85, 77, },{71, 51, 82, 86, 56, 90, 63, 37, },{79, 83, 90, 51, 90, 90, 67, 80, },{90, 90, 90, -47, 65, 64, 40, 90, },},{{90, 90, 77, 87, 84, 69, 57, 37, },{4, 66, 84, 82, 90, 73, 90, 90, },{1, 79, 90, -4, 69, 19, 90, 40, },{68, 84, 39, 84, 78, 55, 81, 77, },},},{{{80, 88, 20, 52, 65, -9, 90, 81, },{90, 77, 80, 45, 43, 57, 90, 90, },{46, 71, 85, 39, 66, 90, 74, 90, },{87, 90, 85, 76, 49, 65, 90, 35, },},{{90, 49, 77, 61, 71, 58, 88, 90, },{50, 62, 7, -15, 48, 90, 90, 70, },{73, -14, 44, 90, 76, 90, 90, 84, },{53, 31, 38, 90, 90, 90, 76, 64, },},{{79, 23, 73, 89, 68, -41, 65, 90, },{66, 69, 88, 81, 88, 44, 64, 90, },{-25, 44, 37, 47, 65, 67, -24, 90, },{14, 60, 90, 2, 34, 13, 73, 87, },},{{51, 22, 35, 53, 58, 88, 24, 64, },{76, 27, 44, 90, 63, 90, 90, 70, },{57, 69, 90, 44, -44, 52, 90, 74, },{48, 79, 56, 33, 27, 77, 90, 90, },},},},{{{{54, 76, 90, 90, 56, -4, 74, 63, },{88, 72, 90, 48, -8, 82, -14, 58, },{67, 65, 90, 79, 64, 90, 56, 0, },{40, 70, 42, 90, 82, 33, 64, -2, },},{{70, 68, 12, 23, 48, 76, -25, 90, },{77, 27, 69, 72, 63, 90, 61, -7, },{90, 28, 47, -11, 90, 11, 73, 65, },{31, 80, 85, -15, 78, -3, 2, 90, },},{{57, 77, 90, 68, 26, 66, 46, 38, },{90, 50, 60, 21, 90, 57, 72, 36, },{-37, 72, 90, 16, 77, 53, 60, 52, },{65, 24, 33, 71, 81, 90, 90, 74, },},{{27, 30, 43, 37, 90, 82, 86, 72, },{90, 78, 76, 32, 25, -6, 64, 90, },{-27, 84, 78, 53, -6, 77, 54, 74, },{30, 54, 74, 41, 66, 31, 90, 65, },},},{{{74, 17, 90, 90, 43, 55, 63, 16, },{4, -3, 80, 90, 90, 9, 90, 80, },{78, 90, 89, 34, 45, 66, 90, 61, },{87, 90, 90, 73, 59, 65, 67, 30, },},{{83, 74, 66, 15, 87, 75, 9, 90, },{58, 58, 29, 82, 84, 90, 86, -4, },{90, 90, 61, 12, 90, 53, 79, 63, },{90, 27, 90, 74, 90, 90, 52, 74, },},{{90, 51, 44, 64, 32, 35, 90, 90, },{66, 62, 42, 33, 70, 51, 72, 81, },{90, 81, 62, 90, 34, 47, 90, 35, },{74, 33, 58, 80, 35, 56, -40, 81, },},{{87, 76, 41, 29, 62, 41, 90, 68, },{58, -6, 32, 90, 31, 65, 90, 90, },{54, 82, 90, 74, 90, 83, 77, 14, },{52, 90, 90, 31, 90, 90, 52, 57, },},},{{{52, 83, 9, 90, 79, 8, -4, 42, },{90, -7, 76, 90, 90, 90, -16, 90, },{90, 90, -6, 77, 86, 70, 68, 72, },{62, 90, 42, 50, 60, 76, 90, 39, },},{{51, 57, 82, -12, 73, 84, 28, 90, },{3, 3, 90, 85, 31, 68, 79, 71, },{90, 46, 17, 50, 59, 49, 68, 85, },{59, 47, 51, 42, 90, 13, 38, 90, },},{{85, 90, 90, 13, 46, 90, -23, 67, },{71, 48, 78, 65, 70, 29, 57, 57, },{55, 15, 72, 71, 76, 47, 62, 60, },{90, 82, 37, 9, 90, 68, 89, 73, },},{{9, 78, 56, 54, 55, 73, 90, 33, },{90, 78, 50, 73, 69, 53, 90, 75, },{19, 34, 57, 28, 63, 75, 27, 2, },{56, 45, 61, 38, 51, 84, 17, -26, },},},{{{68, 90, 65, 70, 88, 45, 78, 85, },{-3, 61, 83, 90, 60, 70, 90, 72, },{60, 24, 90, 72, 46, -20, 49, 64, },{74, 65, 83, 80, 90, 70, 20, 78, },},{{82, 90, 15, 90, 25, 46, 68, 90, },{75, 19, 81, 90, 42, 90, 72, 79, },{23, 70, 77, 50, 36, 90, 76, -14, },{66, 59, 73, 17, 72, 23, 53, -12, },},{{29, 90, 60, 88, 6, 21, 86, 56, },{40, 39, 41, 69, 89, 66, 89, 31, },{47, 90, 65, 37, 28, 18, 90, 82, },{26, 49, 30, 75, 84, 56, 11, 42, },},{{71, 12, 66, 19, 88, 65, 90, 45, },{86, 65, 59, 34, 82, 90, -25, 43, },{55, 90, 90, 89, 73, 70, 90, 39, },{62, 42, 75, 87, 29, 75, 84, 65, },},},},{{{{67, 82, -25, 90, 87, 85, 31, 51, },{80, 90, 28, 7, 54, 90, 87, 40, },{40, 32, 79, 84, 71, 18, 90, 71, },{90, 65, 78, 77, 67, 72, 19, 68, },},{{-29, 77, 4, 70, 82, 54, 90, 90, },{90, 51, 90, 73, 66, 90, 90, 63, },{50, 84, 7, 59, 39, 59, 24, 71, },{68, 55, 90, 90, 61, 90, 2, 81, },},{{90, 65, 73, 29, -71, 90, 20, 57, },{35, 60, 86, 90, 77, 52, 84, -1, },{85, 56, 90, 26, 53, 65, 40, 64, },{-8, 68, 64, 90, 58, 30, 72, 32, },},{{90, 47, 90, 7, 64, 90, 90, 67, },{-26, 33, 30, 28, 27, 75, -10, 90, },{43, 90, 67, 18, 85, 4, 27, 90, },{70, 90, 16, 90, 90, 77, 71, 61, },},},{{{76, 34, 90, 64, 52, 31, 67, 11, },{80, 90, 32, 37, 29, 70, 63, 90, },{69, 75, 90, 5, 90, 79, 64, 78, },{59, 90, 70, 78, 90, 75, 51, 65, },},{{72, 90, 55, 90, 61, 67, 90, 54, },{90, 90, 51, 59, 90, 43, 66, -5, },{90, 39, 85, 39, 85, 90, 76, 65, },{44, 28, 25, 50, 54, 81, 31, 13, },},{{45, 72, 14, 90, 90, 72, 12, 65, },{90, 83, 58, 82, -26, 80, 90, 89, },{60, 56, 90, 86, 43, 51, 40, 90, },{79, 27, 90, 34, 90, 61, 14, 90, },},{{88, 90, 90, 78, 53, 90, 53, 77, },{90, 28, -15, 66, -36, 60, 90, 68, },{90, 75, 77, 63, 54, 90, 70, 20, },{12, 44, 67, 69, 53, 90, 75, 86, },},},{{{17, 90, 90, 90, 90, 55, 32, 90, },{63, -18, 74, 85, 61, 90, 80, 23, },{32, 90, -4, 62, 90, 37, 14, 84, },{90, 77, 90, 80, 77, 76, 90, 54, },},{{81, 11, 52, 85, 42, 84, 32, 6, },{42, 58, 89, 90, 58, 58, 54, 44, },{77, 45, 79, -66, 77, 90, 88, 90, },{43, 90, 11, 61, 69, 73, 72, 61, },},{{43, -4, 81, 90, 90, 71, 90, 53, },{41, 33, 81, 89, 79, 79, 88, 90, },{35, 69, 67, 39, 90, 32, 67, 44, },{84, 73, 78, 23, 90, 90, 56, 87, },},{{28, 90, 86, 65, 61, 45, 69, 67, },{90, -32, 74, 88, 69, 90, 79, 5, },{2, 67, 90, 90, 27, 90, 88, 90, },{90, 37, 90, 12, 90, 44, 52, 90, },},},{{{-12, 82, 90, 61, 53, 40, 84, -1, },{46, 55, 49, 43, 90, 78, 79, 45, },{58, 30, 90, 64, 64, 83, 72, 90, },{86, 48, 24, 52, 87, 72, 83, 81, },},{{49, 57, 72, 28, 79, 90, 90, 77, },{67, -1, 82, -13, 63, 88, 47, 77, },{66, 90, 56, 52, 21, 80, 90, 83, },{78, 72, 85, 56, 89, 90, 62, 68, },},{{28, 67, 90, 74, 55, 42, 65, 55, },{54, 26, 66, 90, 71, 54, 50, 87, },{40, 3, 20, -5, 82, 75, 55, 24, },{77, 83, 71, -55, 90, 89, 90, 90, },},{{90, 72, 23, 89, 55, 54, 73, 88, },{64, 64, 5, 76, 76, 83, 90, 17, },{81, 90, 90, 81, 21, 67, 90, 56, },{-12, -49, 90, 64, 52, 32, 77, 69, },},},},{{{{90, 77, 35, 90, 80, 47, 77, 90, },{54, 80, 62, 52, 45, -18, 59, 62, },{82, 27, 82, 44, 90, 90, 77, 71, },{49, 24, 48, 69, 74, -13, 62, 85, },},{{67, 37, 73, -44, 54, 80, 79, 90, },{31, 90, 34, 90, 90, 39, 66, 77, },{30, 81, 84, 61, -14, 33, 33, 84, },{66, 39, 90, 45, 50, 90, 90, 52, },},{{43, 74, -48, 50, 90, 49, 77, 52, },{61, 25, 71, 22, 90, 73, 25, 90, },{77, 90, 30, 81, 51, 82, 74, 75, },{78, 33, 31, 71, 49, 67, 44, 34, },},{{90, 31, 89, 61, 75, -11, 42, 68, },{66, -25, 84, 60, 80, 90, 48, 48, },{44, 73, 65, -44, 90, 44, 27, 90, },{73, 49, 19, -31, 90, 33, 90, 90, },},},{{{90, 81, 81, -9, -9, -13, 48, 9, },{77, 77, 39, 90, 31, 90, 65, 43, },{-26, 73, 61, 27, 24, 30, 23, 78, },{88, 32, 75, 83, 82, 71, 90, 50, },},{{62, 84, 6, 15, 90, 86, 73, -24, },{83, 43, 62, 47, 3, 57, 43, 36, },{33, 34, 88, 90, 90, 27, 32, 71, },{29, 21, 29, 60, 53, 89, 58, 59, },},{{1, 90, 1, 37, 51, 90, 63, 44, },{74, 42, 80, 22, 56, 57, 47, 36, },{66, 88, 41, 23, 57, 61, 25, 83, },{55, 63, 35, 62, 68, 41, -46, 90, },},{{78, 77, 61, 71, 59, 90, 66, 44, },{-2, 6, 90, 74, 76, 62, -5, 76, },{64, 83, 90, 61, 73, 17, 90, 59, },{22, 30, 10, 54, 90, 87, 51, 71, },},},{{{20, 90, 84, 84, 85, 77, 52, 88, },{64, 14, 42, 27, 55, 41, 90, 89, },{85, 90, 90, -14, 70, 79, 90, 31, },{24, 68, 90, 67, 88, 66, 90, 47, },},{{55, 24, 29, -18, 62, 67, 54, 90, },{53, 78, 87, 81, 34, 32, 80, 61, },{90, 30, 22, 90, 90, -15, 70, 84, },{90, 55, 62, 47, 70, 17, 90, 28, },},{{70, 36, 78, 33, 90, 72, 19, 89, },{78, 90, 17, 32, 70, 75, 79, 43, },{48, 87, 1, 47, 83, 60, 1, 84, },{90, 40, 41, 86, 33, 90, 74, 43, },},{{60, 63, 90, 67, 69, 21, 52, 55, },{77, 17, 4, -31, 70, 72, 83, 70, },{82, 88, 90, 45, 40, 71, 86, 78, },{87, 67, -25, 66, 72, 0, 67, 90, },},},{{{18, 90, 61, 90, 90, 80, 90, 83, },{64, 78, 68, 65, 40, 90, 58, 90, },{57, 90, 61, 90, 62, 80, 26, 75, },{89, 90, 75, 90, 70, 60, 59, 52, },},{{90, 51, 70, 23, -6, 90, 90, 54, },{77, 45, 77, 5, 81, 90, 90, 90, },{90, -65, 81, 14, 90, 47, 90, 88, },{16, 54, 64, 2, 27, 77, 90, 48, },},{{38, 89, 81, 74, 57, -47, 24, 19, },{75, 90, 83, 90, 90, 88, 41, 90, },{8, 73, 10, 74, 52, 23, 4, 13, },{-8, 90, -31, 53, 82, 87, 83, 48, },},{{64, 90, 77, 4, 90, 61, 90, 90, },{48, -15, 74, 79, 17, 73, 68, 74, },{51, 83, 57, 90, 69, 90, 62, 90, },{24, 68, 90, 58, 50, 90, 90, 51, },},},},},};