function x_plus = jump_map(x, param)
qMinus = x(1:2);
dqdtMinus = x(3:4);

qPlus = [qMinus(2); qMinus(1)];

QMinus = Q_minus(qMinus, param);
QPlus = Q_plus(qPlus, param);

dqdtPlus = QPlus\(QMinus*dqdtMinus);

x_plus = [qPlus; dqdtPlus];
end

