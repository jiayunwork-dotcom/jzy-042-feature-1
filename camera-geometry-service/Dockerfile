# ---------- build stage ----------
FROM maven:3.9-eclipse-temurin-17 AS build
WORKDIR /app

# Cache dependencies in their own layer.
COPY pom.xml .
RUN mvn -B -q dependency:go-offline

# Tests run as part of `package`: the image only builds when the suite is green.
COPY src ./src
RUN mvn -B -q package

# ---------- runtime stage ----------
FROM eclipse-temurin:17-jre
WORKDIR /app
COPY --from=build /app/target/camera-geometry-service.jar app.jar
EXPOSE 8080
ENTRYPOINT ["java", "-jar", "app.jar"]
