function stable = VerifyPSD(P, S, D)
    stable = P > 0 && S > -P && -(P + S) < D && D < P;
end